/*
 * Copyright (C) 2021, Xilinx Inc - All rights reserved
 * Xilinx Decoder/Encoder XMA Plugin
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <syslog.h>

#include <xmaplugin.h>
#include <queue>
#include <unordered_set>

#include "mpsoc_vcu_hdr.h"

#undef MEASURE_TIME
#ifdef MEASURE_TIME
#define MAX_COUNT_TIME 1000
#include <time.h>
#endif

#define ENC_REGMAP_SIZE 20

#include <xvbm.h>
/* #define XVBM_BUFF_PR(...) printf(__VA_ARGS__) */
#define XVBM_BUFF_PR(...)

#define CHANNEL_OFFSET    (5)
#define CHANNEL_MAX       (20 + CHANNEL_OFFSET)
#define MAX_CHAN_CAP      ((unsigned long)3840 * 2160 * 60 / 1)
#define NUM_CHANNELS      (CHANNEL_MAX - CHANNEL_OFFSET + 1)
#define NUM_SK_INSTANCES  NUM_ENC_SK_INSTANCES

#define ENC_LOCK_FILE     "/tmp/kernel_vcu_encoder.txt"
#define ENC_SK_PATH       "/lib/firmware/xilinx/kernel_vcu_encoder.so"
#define ENC_SK_NAME       "kernel_vcu_encoder"

#define MAX_OUT_BUFF_COUNT 50

//#define DUMP_QP_MAPS 1
//#define DUMP_FSFA 1
//#define DUMP_INPUT_YUV 1

#define MIN_LOOKAHEAD_DEPTH (1)
#define MAX_LOOKAHEAD_DEPTH (30)

// PIPELINE_DEPTH > 2 is not supported
#define PIPELINE_DEPTH      2

#define VCU_STRIDE_ALIGN    32
/* 'slice-height should be 16-aligned for AVC and 32-aligned for HEVC',
 * but aligning to 32 works for both */
#define VCU_HEIGHT_ALIGN    32
#define ALIGN(x,align) (((x) + (align) - 1) & ~((align) - 1))

/* App using this plugin is suggested to have >= below size.
   MAX_EXTRADATA_SIZE should be consistent with AL_ENC_MAX_CONFIG_HEADER_SIZE on device */
#define MAX_EXTRADATA_SIZE  (2 * 1024)
#define MAX_ERR_STRING      1024
#define XMA_VCU_ENCODER     "xma-vcu-encoder"
#define WARN_BUFF_MAX_SIZE  (4 * 1024)
static int32_t xma_encoder_close(XmaEncoderSession *enc_session);

enum cmd_type
{
    VCU_PREINIT = 0,
    VCU_INIT,
    VCU_PUSH,
    VCU_RECEIVE,
    VCU_FLUSH,
    VCU_DEINIT,
};

enum dev_cu_status
{
    SK_STATUS_OKAY,
    SK_STATUS_BAD
};

enum rc_mode
{
    AL_RC_CONST_QP = 0x00,
    AL_RC_CBR = 0x01,
    AL_RC_VBR = 0x02,
    AL_RC_LOW_LATENCY = 0x03,
    AL_RC_CAPPED_VBR = 0x04,
    AL_RC_BYPASS = 0x3F,
    AL_RC_PLUGIN = 0x40,
    AL_RC_MAX_ENUM,
};

typedef struct xlnx_rc_fsfa
{
    uint32_t fs;
    uint32_t fa;
} xlnx_rc_fsfa_t;

typedef struct enc_params
{
    char *enc_options;
    char *lambda_file;
} enc_static_params_t;

typedef struct enc_dynamic_params
{
    uint16_t    width;
    uint16_t    height;
    double      framerate;
    uint16_t    rc_mode;
} enc_dynamic_params_t;

typedef struct _vcu_enc_usermeta
{
    int64_t pts;
    int frame_type;
} vcu_enc_usermeta;

typedef struct __obuf_info
{
    uint32_t obuff_index;
    uint32_t recv_size;
    vcu_enc_usermeta obuf_meta;
} obuf_info;

typedef struct enc_dyn_params
{
    bool is_bitrate_changed;
    uint32_t bit_rate;
    bool is_bframes_changed;
    uint8_t num_b_frames;
}enc_dyn_params_t;

typedef struct host_dev_data
{
    uint32_t cmd_id;
    uint32_t cmd_rsp;
    uint32_t ibuf_size;
    uint32_t ibuf_count;
    uint32_t ibuf_index;
    uint64_t ibuf_paddr;
    uint32_t qpbuf_size;
    uint32_t qpbuf_count;
    uint32_t qpbuf_index;
    uint32_t obuf_size;
    uint32_t obuf_count;
    uint32_t freed_ibuf_index;
    uint32_t freed_qpbuf_index;
    uint16_t extradata_size;
    vcu_enc_usermeta ibuf_meta;
    obuf_info obuf_info_data[MAX_OUT_BUFF_COUNT];
    uint32_t freed_index_cnt;
    uint32_t obuf_indexes_to_release[MAX_OUT_BUFF_COUNT];
    uint32_t obuf_indexes_to_release_valid_cnt;
    bool is_idr;
    bool end_encoding;
    bool is_dyn_params_valid;
    enc_dyn_params_t dyn_params;
    uint8_t extradata[MAX_EXTRADATA_SIZE];
    uint32_t frame_sad[MAX_LOOKAHEAD_DEPTH];
    uint32_t frame_activity[MAX_LOOKAHEAD_DEPTH];
    uint32_t la_depth;
    char dev_err[MAX_ERR_STRING];
    uint32_t stride_width;
    uint32_t stride_height;
    uint32_t warn_buf_size;
#ifdef HDR_DATA_SUPPORT
    vcu_hdr_data hdr_data;
#endif
   bool duplicate_frame;
} sk_payload_data;

typedef struct buff_pool_ctx
{
    XmaBufferObj parent_buff_obj;
    uint64_t    parent_buff_paddr;
    XmaBufferObj *buf_objs;
    uint32_t    buf_size;
    uint32_t    buf_count;
    uint32_t    current_free_obuf_index;
    uint32_t    free_index_to_device;
    std::queue <uint32_t> *obuf_free_list;
    sk_payload_data *dev_data;
} buff_pool_ctx_t;

typedef struct xrt_cmd_data
{
    uint32_t  registers[ENC_REGMAP_SIZE];
    size_t size;
    XmaBufferObj payload_buf_obj;
    uint64_t payload_buf_paddr;
    XmaSession *xma_session;
    XvbmBufferHandle b_handle[PIPELINE_DEPTH];
    XvbmPoolHandle pool_handle;
    buff_pool_ctx_t *ibuf_ctx;
    buff_pool_ctx_t *qpbuf_ctx;
    buff_pool_ctx_t *obuf_ctx;
    uint32_t qpbuf_idx;
    XmaBufferObj params_buff_obj;
    uint64_t params_buff_paddr;
    uint8_t *params_buff;
    size_t params_size;
    XmaEncoderProperties *enc_props;
    XmaBufferObj dynamic_params_buff_obj;
    uint64_t dynamic_params_buff_paddr;
    enc_dynamic_params_t *dynamic_params_buff;
    enc_static_params_t params_data;
    XmaBufferObj lambda_buff_obj;
    uint64_t lambda_buff_paddr;
    uint8_t *lambda_buff;
    size_t lambda_size;
    bool is_idr[PIPELINE_DEPTH];
    xlnx_rc_fsfa_t *fsfa_ptr;
    uint32_t fsfa_num;
    XmaSideDataHandle rc_sd_handle;
    dev_cu_status cu_status;
    uint64_t timestamp;
    uint32_t stride_width;
    uint32_t stride_height;
    XmaBufferObj warn_buff_obj;
    uint64_t warn_buff_paddr;
#ifdef HDR_DATA_SUPPORT
    bool hdr_data_present;
#endif
    bool duplicate_frame;
} xrt_cmd_data_t;

typedef struct MpsocEncContext
{
    int m_frame;
    uint32_t m_curr_qpbuf_idx;
    xrt_cmd_data_t  *sk_cmd;
    bool initial_ibufs_consumed;
    bool initial_qpbufs_consumed;
    XvbmPoolHandle pool;
    bool pool_extended;
#ifdef MEASURE_TIME
    int send_count;
    int recv_count;
    long long int send_func_time;
    long long int recv_func_time;
    long long int send_xrt_time;
    long long int recv_xrt_time;
#endif
    long long int frame_sent;
    long long int frame_recv;
    struct timespec latency;
    long long int time_taken;
    int latency_logging;
    uint32_t first_frame;
    void *enc_recv_out_databuf;
    int32_t enc_recv_out_data_size;
    int64_t enc_recv_out_data_pts;
    int32_t enc_recv_status;
    int32_t enc_recv_free_obuf_idx;
    bool enc_recv_eos;
    bool read_last_frame;
    bool clear_sb_data;
    bool cmd_scheduled;
    uint32_t pipeline_widx;
    uint32_t pipeline_ridx;
    enum cmd_type cmd[PIPELINE_DEPTH];
    int64_t frame_pts[PIPELINE_DEPTH];
    uint32_t enable_hw_in_buf;
    uint32_t hdr_frame_count;
    // sk does not return unused buffers, when flushed
    std::unordered_multiset<XvbmBufferHandle> *pending_release;
} MpsocEncContext;

enum
{
    STATIC_PARAM_FILE,
    LAMBDA_FILE,
};

static int32_t run_recv_cmd(MpsocEncContext *ctx, XmaSession  xma_session);
static int prepare_sk_cmd(xrt_cmd_data_t *cmd, enum cmd_type type, int64_t pts,
                          uint32_t pipeline_idx);
static int get_raw_host_frame(MpsocEncContext *ctx, XmaFrame *frame,
                              uint32_t widx);
static int get_qp_map_data(MpsocEncContext *ctx, XmaFrame *frame);
static int get_custom_rc_data(MpsocEncContext *ctx, XmaFrame *frame);
static int get_sideband_data(MpsocEncContext *ctx, XmaFrame *frame);
static int process_cmd_response(MpsocEncContext *ctx, XmaSession  xma_session);
static void zero_out_padded_region(uint8_t *dst,
                                   int16_t start_x,
                                   int16_t start_y,
                                   int16_t dst_stride,
                                   int16_t dst_height);
#ifdef HDR_DATA_SUPPORT
static int send_hdr_data_to_sk(XmaEncoderSession *enc_session, XmaFrame *frame)
{
    MpsocEncContext  *ctx  = (MpsocEncContext *)enc_session->base.plugin_data;
    vcu_hdr_data *pHDRdata = &ctx->sk_cmd->obuf_ctx->dev_data->hdr_data;

    XmaSideDataHandle hdr_sd    = NULL;
    uint8_t          *sd_ptr    = NULL;
    size_t            sd_size   = 0;

    hdr_sd = xma_frame_get_side_data(frame, XMA_FRAME_HDR);
    xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER,"Sidedata ref count of XMA_FRAME_HDR = %d\n",xma_side_data_get_refcount(hdr_sd));
    if (hdr_sd) {
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_HDR,"Detected HDR side data in encoder input frame \n");
        ctx->sk_cmd->hdr_data_present = 1;
        sd_ptr  = (uint8_t *)xma_side_data_get_buffer(hdr_sd);
        sd_size = xma_side_data_get_size(hdr_sd);
        memcpy(pHDRdata, sd_ptr, sd_size*sizeof(char));
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER, "HDR data in encoder - %d  \n", ++ctx->hdr_frame_count);
        print_hdr_data(pHDRdata);
        xma_frame_remove_side_data_type(frame, XMA_FRAME_HDR);
    }
    return XMA_SUCCESS;

}
#endif

static XmaParameter *getParameter (XmaParameter *params, int num_params,
                                   const char  *name)
{
    int i = 0;

    for (i = 0; i < num_params; ++i) {
        if (!strcmp(name, params[i].name)) {
            return &params[i];
        }
    }

    return NULL;
}

static int32_t run_xrt_cmd(xrt_cmd_data_t *cmd)
{
    int ret;

    if (!cmd || (cmd->cu_status == SK_STATUS_BAD)) {
        return XMA_ERROR;
    }

    XmaCUCmdObj cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                         (char *)(cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);

    while (ret < 0) {
        //sleep(1);
        cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                                            (char *)(cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);
    };

    ret = xma_plg_is_work_item_done(*(cmd->xma_session), 5000);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "*****ERROR:: Encoder stopped responding (%s)*****\n", __func__);
        cmd->cu_status = SK_STATUS_BAD;
        return ret;
    }

    return XMA_SUCCESS;
}


static int32_t schedule_cmd(xrt_cmd_data_t *cmd, enum cmd_type type,
                            int64_t frame_pts, uint32_t idx)
{
    int ret;

    if (!cmd) {
        return XMA_ERROR;
    }

    ret = prepare_sk_cmd(cmd, type, frame_pts, idx);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) prepare_sk_cmd failed\n", __func__);
        return XMA_ERROR;
    }

    XmaCUCmdObj cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                         (char *)(cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);

    while (ret < 0) {
        cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                                            (char *)(cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);
    };

    return XMA_SUCCESS;
}

static void free_xrt_res(xrt_cmd_data *cmd)
{
    if (cmd) {
        free(cmd);
    }
}

static void free_bufpool_resources(buff_pool_ctx_t  *buf_ctx,
                                   XmaSession xma_session)
{
    uint32_t i;

    if (!buf_ctx) {
        return;
    }

    for (i = 0; i < buf_ctx->buf_count; i++) {
        if (buf_ctx->buf_objs[i].data) {
            xma_plg_buffer_free (xma_session, buf_ctx->buf_objs[i]);
        }
    }

    if (buf_ctx->buf_objs->data) {
        free(buf_ctx->buf_objs);
    }

    if (buf_ctx->parent_buff_obj.data) {
        xma_plg_buffer_free (xma_session, buf_ctx->parent_buff_obj);
    }

    free(buf_ctx);
}

static int allocate_buff_pool(XmaSession xma_session,
                              struct buff_pool_ctx *buf_ctx)
{
    int ret, j;
    uint32_t i;
    uint64_t paddr;
    size_t offset = 0;
    ret = XMA_SUCCESS;

    buf_ctx->buf_objs = (XmaBufferObj *) malloc (buf_ctx->buf_count * sizeof (
                            XmaBufferObj));
    if (!buf_ctx->buf_objs) {
        return XMA_ERROR;
    }

    buf_ctx->parent_buff_obj = xma_plg_buffer_alloc(xma_session,
                               buf_ctx->buf_count * sizeof (uint64_t), false, &ret);
    if(!buf_ctx->parent_buff_obj.data) {
        free(buf_ctx->buf_objs);
        return XMA_ERROR;
    }

    buf_ctx->parent_buff_paddr =  buf_ctx->parent_buff_obj.paddr;
    if (!buf_ctx->parent_buff_paddr) {
        free(buf_ctx->buf_objs);
        xma_plg_buffer_free(xma_session, buf_ctx->parent_buff_obj);
        return XMA_ERROR;
    }

    for (i = 0 ; i < buf_ctx->buf_count; i++) {
        buf_ctx->buf_objs[i] = xma_plg_buffer_alloc(xma_session, buf_ctx->buf_size,
                               false, &ret);
        if (ret != XMA_SUCCESS) {
            ret = XMA_ERROR;
            break;
        }


        paddr =  buf_ctx->buf_objs[i].paddr;
        memcpy(buf_ctx->parent_buff_obj.data+offset, &paddr, sizeof (uint64_t));
        ret = xma_plg_buffer_write (xma_session, buf_ctx->parent_buff_obj,
                                    sizeof (uint64_t), offset);
        if (ret != XMA_SUCCESS) {
            return ret;
        }

        offset += sizeof (uint64_t);
    }

    if (ret != XMA_SUCCESS) {
        /* free earlier handles */
        for (j=i; j>=0; j--) {
            if (buf_ctx->buf_objs[j].data) {
                xma_plg_buffer_free(xma_session, buf_ctx->buf_objs[j]);
            }
        }
        xma_plg_buffer_free(xma_session, buf_ctx->parent_buff_obj);
    }

    return ret;
}

static int read_static_configuration(xrt_cmd_data_t *cmd, const char *in_file,
                                     int type)
{
    int fd, ret;
    unsigned long size = 0;
    struct stat buf;
    XmaBufferObj buff_obj;
    uint64_t buff_paddr;
    uint8_t *buff;

    fd = 0;
    if (type != STATIC_PARAM_FILE && type != LAMBDA_FILE) {
        return XMA_ERROR;
    }

    if (type == LAMBDA_FILE) {
        fd = open(in_file, O_RDONLY, 755);
        if (fd < 0) {
            perror("cannot open file");
            return XMA_ERROR;
        }

        ret = fstat(fd, &buf);
        if (ret < 0) {
            perror("unable to get config file information");
            goto err1;
        }
        size = buf.st_size;
    } else if (type == STATIC_PARAM_FILE) {
        size = strlen(in_file);
    }

    buff_obj = xma_plg_buffer_alloc(*(cmd->xma_session), size, false, &ret);
    if (ret != XMA_SUCCESS) {
        goto err1;
    }

    buff_paddr = buff_obj.paddr;
    if (!buff_paddr) {
        goto err2;
    }

    buff = buff_obj.data;
    if (buff == MAP_FAILED) {
        goto err2;
    }

    if (type == LAMBDA_FILE) {
        ret = read(fd, buff, buf.st_size);
        if (ret < 0) {
            goto err3;
        }
    } else if (type == STATIC_PARAM_FILE) {
        strcpy ((char *)buff, in_file);
    }

//  ret = xma_plg_buffer_write (*(cmd->xma_session), buff_obj,buf.st_size, 0);//ALERT:TODO:DG02252020:for staticparam file it seems wrong to use buf.st_size.Need to check
    ret = xma_plg_buffer_write (*(cmd->xma_session), buff_obj, size, 0);
    if (ret != XMA_SUCCESS) {
        return ret;
    }

    switch (type) {
        case STATIC_PARAM_FILE:
            cmd->params_buff_obj    = buff_obj;
            cmd->params_buff_paddr  = buff_obj.paddr;
            cmd->params_buff        = buff_obj.data;
            /* //ALERT:temp change to see if data is received proper
                  cmd->params_buff_obj    = buff_obj;
                  cmd->params_buff_paddr  = buff_paddr;
                  cmd->params_buff        = buff;
            */
            cmd->params_size        = size;
            break;
        case LAMBDA_FILE:
            cmd->lambda_buff_obj = buff_obj;
            cmd->lambda_buff_paddr  = buff_paddr;
            cmd->lambda_buff        = buff;
            cmd->lambda_size        = size;
            close(fd);
            break;
        default:
            /* shouldn't reach here */
            break;
    }

    return XMA_SUCCESS;

err3:
    munmap(buff, buf.st_size);
err2:
    xma_plg_buffer_free(*(cmd->xma_session), buff_obj);
err1:
    if (type == LAMBDA_FILE) {
        close(fd);
    }
    return XMA_ERROR;
}

static uint16_t set_rc_mode(const int32_t rate_control_mode)
{
    uint16_t rc_mode = AL_RC_CONST_QP;
    if (rate_control_mode == 1) {
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER,
                   "Encoder RC Mode set to Custom RC\n");
        rc_mode = AL_RC_PLUGIN;
    }
    return rc_mode;
}

static int prepare_sk_cmd(xrt_cmd_data_t *cmd, enum cmd_type type, int64_t pts,
                          uint32_t pipeline_idx)
{
    int ret;
    int idx = 0;
    sk_payload_data data;

    ret = XMA_SUCCESS;

    if (!cmd || cmd->cu_status == SK_STATUS_BAD) {
        return XMA_ERROR;
    }

    memset(&data, 0, sizeof(sk_payload_data));

    cmd->registers[idx]    = 0;
    cmd->registers[++idx]  = type;
    cmd->registers[++idx]  = getpid();
    cmd->registers[++idx]  = cmd->timestamp & 0xFFFFFFFF;
    cmd->registers[++idx]  = (cmd->timestamp  >> 32) & 0xFFFFFFFF;

    cmd->registers[++idx] = cmd->payload_buf_paddr & 0xFFFFFFFF;
    cmd->registers[++idx] = ((uint64_t)(cmd->payload_buf_paddr) >> 32) & 0xFFFFFFFF;
    cmd->registers[++idx] = sizeof(sk_payload_data);

    data.cmd_id = type;
    switch (type) {
        case VCU_INIT:
            if (!cmd->obuf_ctx->buf_size) {
                ret = XMA_ERROR;
                break;
            }
            /*TODO: After Zerocopy 4,5,6 offset are not used */
            cmd->registers[++idx] = 0;
            cmd->registers[++idx] = 0;
            cmd->registers[++idx] = 0;

            /* ouput buffer pool allocation */
            ret = allocate_buff_pool(*(cmd->xma_session), cmd->obuf_ctx);
            if (ret != XMA_SUCCESS) {
                if (cmd->ibuf_ctx) {
                    free (cmd->ibuf_ctx);
                }
                cmd->ibuf_ctx = nullptr;
                break;
            }

            cmd->registers[++idx] = cmd->obuf_ctx->parent_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->obuf_ctx->parent_buff_paddr) >> 32) &
                                    0xFFFFFFFF;
            cmd->registers[++idx] = cmd->obuf_ctx->buf_count * sizeof (uint64_t);

            if (cmd->qpbuf_ctx) {
                /* qp buffer pool allocation */
                ret = allocate_buff_pool(*(cmd->xma_session), cmd->qpbuf_ctx);
                if (ret != XMA_SUCCESS) {
                    free_bufpool_resources(cmd->obuf_ctx, *(cmd->xma_session));
                    cmd->obuf_ctx = nullptr;
                    if (cmd->ibuf_ctx) {
                        free (cmd->ibuf_ctx);
                    }
                    cmd->ibuf_ctx = nullptr;
                    break;
                }

                cmd->registers[++idx] = cmd->qpbuf_ctx->parent_buff_paddr & 0xFFFFFFFF;
                cmd->registers[++idx] = ((uint64_t)(cmd->qpbuf_ctx->parent_buff_paddr) >> 32) &
                                        0xFFFFFFFF;
                cmd->registers[++idx] = cmd->qpbuf_ctx->buf_count * sizeof (uint64_t);
            } else {
                /* change count as per VCU_INIT command needs */

            }
            break;
        case VCU_PUSH:
            uint32_t i;
            data.is_idr = cmd->is_idr[pipeline_idx];
            data.is_dyn_params_valid = cmd->obuf_ctx->dev_data->is_dyn_params_valid;
            if(data.is_dyn_params_valid) {
                memcpy(&data.dyn_params, &cmd->obuf_ctx->dev_data->dyn_params, sizeof(enc_dyn_params_t));
                cmd->obuf_ctx->dev_data->is_dyn_params_valid = 0;
            }

            /* initial input buff index to send*/
            data.ibuf_size  = xvbm_buffer_get_size(cmd->b_handle[pipeline_idx]);
            data.ibuf_index = xvbm_buffer_get_id(cmd->b_handle[pipeline_idx]);
            data.ibuf_paddr = xvbm_buffer_get_paddr(cmd->b_handle[pipeline_idx]);
            data.ibuf_meta.pts = pts;
            data.duplicate_frame = cmd->duplicate_frame;
#ifdef HDR_DATA_SUPPORT
            if(cmd->hdr_data_present) {
                memcpy(&data.hdr_data, &cmd->obuf_ctx->dev_data->hdr_data, sizeof(vcu_hdr_data));
                cmd->hdr_data_present = 0;
            }
#endif

            i = 0;
            while(cmd->obuf_ctx->obuf_free_list->size()) {
                data.obuf_indexes_to_release[i] = cmd->obuf_ctx->obuf_free_list->front();
                cmd->obuf_ctx->obuf_free_list->pop();
                i++;
            }
            data.obuf_indexes_to_release_valid_cnt = i;

            XVBM_BUFF_PR("Enc: Received Input Buffer: Index = %d\n", data.ibuf_index);

            if (cmd->qpbuf_ctx) {
                data.qpbuf_index = cmd->qpbuf_idx;
            }

            /*
             * If RC Ptr setup,
             * then pass down to soft-kernel
             */
            if (cmd->fsfa_ptr) {
                if (cmd->fsfa_num < MIN_LOOKAHEAD_DEPTH ||
                        cmd->fsfa_num > MAX_LOOKAHEAD_DEPTH) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                               "ERROR: LA Depth %d is Unsupported for Custom RC\n", cmd->fsfa_num);
                    if (cmd->rc_sd_handle) {
                        xma_side_data_dec_ref(cmd->rc_sd_handle);
                    }
                    return XMA_ERROR;
                }
                /* Custom RC */
                for (unsigned int num = 0; num < cmd->fsfa_num; num++) {
                    data.frame_sad[num] = cmd->fsfa_ptr[num].fs;
                    data.frame_activity[num] = cmd->fsfa_ptr[num].fa;
                }

                data.la_depth = cmd->enc_props->lookahead_depth;

                if (cmd->rc_sd_handle) {
                    xma_side_data_dec_ref(cmd->rc_sd_handle);
                }

            } else {
                /* Default RC */
                memset(data.frame_sad, 0, MAX_LOOKAHEAD_DEPTH * sizeof data.frame_sad[0]);
                memset(data.frame_activity, 0,
                       MAX_LOOKAHEAD_DEPTH * sizeof data.frame_activity[0]);
                data.la_depth = 0;
            }
            data.stride_width  = cmd->stride_width;
            data.stride_height = cmd->stride_height;
            break;
        case VCU_PREINIT:
            /* TODO: get file permission flags for 755 */
            ret = read_static_configuration(cmd, cmd->params_data.enc_options,
                                            STATIC_PARAM_FILE);
            if (ret != XMA_SUCCESS) {
                return ret;
            }

            cmd->registers[++idx] = cmd->params_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->params_buff_paddr) >> 32) & 0xFFFFFFFF;
            cmd->registers[++idx] = cmd->params_size;

            /* dynamic encoder paamenters from upstream plugin */
            cmd->dynamic_params_buff_obj = xma_plg_buffer_alloc(*(cmd->xma_session),
                                           sizeof(enc_dynamic_params_t), false, &ret);
            if (ret != XMA_SUCCESS) {
                cmd->dynamic_params_buff = nullptr;
                goto err1;
            }
            cmd->dynamic_params_buff_paddr = cmd->dynamic_params_buff_obj.paddr;
            cmd->dynamic_params_buff = (enc_dynamic_params_t *)
                                       cmd->dynamic_params_buff_obj.data;
            if (cmd->dynamic_params_buff == MAP_FAILED) {
                goto err3;
            }

            cmd->dynamic_params_buff->width = cmd->enc_props->width;
            cmd->dynamic_params_buff->height = cmd->enc_props->height;
            cmd->dynamic_params_buff->rc_mode = set_rc_mode(cmd->enc_props->rc_mode);
            /* non zero denominator is prechecked, before we use it below */
            cmd->dynamic_params_buff->framerate = (cmd->enc_props->framerate.numerator *
                                                   1.0) /
                                                  (cmd->enc_props->framerate.denominator);

#ifdef MEASURE_TIME
            printf (": w = %d, h = %d, fps = %f rc_mode = %s\n",
                    cmd->dynamic_params_buff->width, cmd->dynamic_params_buff->height,
                    cmd->dynamic_params_buff->framerate,
                    (cmd->dynamic_params_buff->rc_mode == AL_RC_PLUGIN ? "Custom RC" : "Default"));
#endif

            ret = xma_plg_buffer_write (*(cmd->xma_session), cmd->dynamic_params_buff_obj,
                                        sizeof(enc_dynamic_params_t), 0);
            if (ret != 0) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "xclSyncBO failed %d in %s \n", ret,
                           __func__);
                goto err3;
            }

            cmd->registers[++idx] = cmd->dynamic_params_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->dynamic_params_buff_paddr) >> 32) &
                                    0xFFFFFFFF;
            cmd->registers[++idx] = sizeof(enc_dynamic_params_t);

            /* static lambda hex configuration from file - optional */
            if (cmd->params_data.lambda_file) {
                ret = read_static_configuration(cmd, cmd->params_data.lambda_file, LAMBDA_FILE);
                if (ret != XMA_SUCCESS) {
                    goto err3;
                }

            } else {
                cmd->lambda_buff_paddr = 0;
                cmd->lambda_size       = 0;
            }

            cmd->registers[++idx] = cmd->lambda_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->lambda_buff_paddr) >> 32) & 0xFFFFFFFF;
            cmd->registers[++idx] = cmd->lambda_size;

            cmd->warn_buff_obj = xma_plg_buffer_alloc(*(cmd->xma_session),
                                         WARN_BUFF_MAX_SIZE, false, &ret);
            if (ret != XMA_SUCCESS)
                goto err3;

            cmd->warn_buff_paddr = cmd->warn_buff_obj.paddr;

            cmd->registers[++idx] = cmd->warn_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->warn_buff_paddr) >> 32) & 0xFFFFFFFF;
            cmd->registers[++idx] = WARN_BUFF_MAX_SIZE;

            ret = XMA_SUCCESS;
            goto done;
err3:
            //munmap(cmd->dynamic_params_buff, sizeof(enc_dynamic_params_t));
            xma_plg_buffer_free(*(cmd->xma_session), cmd->dynamic_params_buff_obj);
            cmd->dynamic_params_buff = nullptr;
err1:
            //munmap(cmd->params_buff, cmd->params_size);
            xma_plg_buffer_free(*(cmd->xma_session), cmd->params_buff_obj);
            ret = XMA_ERROR;
done:
            break;
        case VCU_RECEIVE:
            break;
        case VCU_FLUSH:
        case VCU_DEINIT:
            /* do nothing */
            usleep(30000);// 30msec sleep added for NULL frame used during flush
            break;
    }

    if (ret == XMA_SUCCESS) {
        memcpy (cmd->payload_buf_obj.data, &data, sizeof(sk_payload_data));
        ret = xma_plg_buffer_write (*(cmd->xma_session), cmd->payload_buf_obj,
                                    sizeof(sk_payload_data), 0);
        if (ret != 0) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "xclSyncBO failed %d in %s \n", ret,
                       __func__);
            return XMA_ERROR;
        }
    }

    return ret;
}

static int32_t xma_encoder_init(XmaEncoderSession *enc_session)
{
    int ret;
    XmaParameter *param;
    sk_payload_data payload_data;
    struct timespec init_time;

    if(*(uint8_t *)enc_session->base.plugin_data != 0) {
        return XMA_ERROR;
    }

    openlog ("XMA_Encoder", LOG_PID, LOG_USER);


    MpsocEncContext  *ctx  = (MpsocEncContext *) enc_session->base.plugin_data;
    XmaEncoderProperties *props = &enc_session->encoder_props;
    XmaSession  xma_session  = enc_session->base;

    if ((props->width % 2) || (props->height % 2)) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "error : unsupported resolution\n");
        return XMA_ERROR;
    }

    syslog (LOG_DEBUG, "xma_enc_handle = %p\n", ctx);
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog (LOG_DEBUG, "%s : %p : xma_encoder init start at %lld \n", __func__, ctx,
            ctx->time_taken);

    ctx->m_frame = 0;
    ctx->m_curr_qpbuf_idx = 0;

    if(PIPELINE_DEPTH > 2) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Pipeline Depth > 2 not supported\n");
        return XMA_ERROR;
    }

    /* configure soft kernel - command structure */
    xrt_cmd_data_t *sk_cmd = (xrt_cmd_data *) malloc(sizeof(xrt_cmd_data_t));
    if (!sk_cmd) {
        return XMA_ERROR;
    }

    memset (sk_cmd, 0, sizeof(xrt_cmd_data_t));
    ctx->sk_cmd = sk_cmd;

    sk_cmd->cu_status = SK_STATUS_OKAY;

    /* get encode properties populated from application/plugin user */
    sk_cmd->enc_props = &enc_session->encoder_props;
    if (!(param = getParameter (props->params, props->param_cnt, "enc_options"))) {
        free(sk_cmd);
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "cannot get encoder options parameter\n");
        return XMA_ERROR;
    }

    sk_cmd->params_data.enc_options = *(char **)param->value;

    if ((param = getParameter (props->params, props->param_cnt,
                               "latency_logging"))) {
        ctx->latency_logging = *(uint32_t *)param->value;
    } else {
        ctx->latency_logging = 0;
    }

    /* get lambda hex file from application/plugin user */
    if (!(param = getParameter (props->params, props->param_cnt, "lambda_file"))) {
        sk_cmd->params_data.lambda_file = NULL;
    } else {
        sk_cmd->params_data.lambda_file = *(char **)param->value;
    }
    sk_cmd->size = 1024 * sizeof(int);
    sk_cmd->payload_buf_obj = xma_plg_buffer_alloc (xma_session,
                              sizeof(sk_payload_data), false, &ret);
    if (ret != XMA_SUCCESS) {
        free(sk_cmd);
        return XMA_ERROR;
    }

    sk_cmd->payload_buf_paddr =  sk_cmd->payload_buf_obj.paddr;
    sk_cmd->xma_session = &xma_session;

#ifdef MEASURE_TIME
    xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER, "ctx %p ", ctx);
#endif

    clock_gettime (CLOCK_MONOTONIC, &init_time);
    sk_cmd->timestamp = ((init_time.tv_sec * 1e6) + (init_time.tv_nsec/1e3));
    /* execute VCU_PREINIT command */
    ret = prepare_sk_cmd(sk_cmd, VCU_PREINIT, -1, 0);
    if (ret != XMA_SUCCESS) {
        goto err3;
    }

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        goto err4;
    }

    /* get input and output buffer configuration */

    bzero(&payload_data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(xma_session, sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);

    if (ret != XMA_SUCCESS) {
        goto err4;
    }
    memcpy( &payload_data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

    if (!payload_data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "VCU_PREINIT failed : %s\n",
                   payload_data.dev_err);
        xma_encoder_close(enc_session);
        return XMA_ERROR;
    }

    if (payload_data.warn_buf_size && payload_data.warn_buf_size < WARN_BUFF_MAX_SIZE) {
       memset(sk_cmd->warn_buff_obj.data, 0, WARN_BUFF_MAX_SIZE);
       ret = xma_plg_buffer_read(xma_session, sk_cmd->warn_buff_obj,
		                 payload_data.warn_buf_size, 0);

       if (ret != XMA_SUCCESS) {
           goto err4;
       }
       xma_logmsg(XMA_WARNING_LOG, XMA_VCU_ENCODER, "device warning: %s", sk_cmd->warn_buff_obj.data);
    }

    sk_cmd->ibuf_ctx = (buff_pool_ctx_t *)malloc (sizeof (buff_pool_ctx_t));
    if (!(sk_cmd->ibuf_ctx)) {
        goto err4;
    }

    if ((param = getParameter (props->params, props->param_cnt,
                               "enable_hw_in_buf"))) {
        ctx->enable_hw_in_buf = *(uint32_t *)param->value;
    } else {
        ctx->enable_hw_in_buf = 0;
    }
    ctx->pool_extended   = false;

//  ctx->pool = xvbm_buffer_pool_create_by_device_id (sk_cmd->xma_session->hw_session.dev_index, payload_data.ibuf_count, payload_data.ibuf_size, 0);
    if (ctx->enable_hw_in_buf == 0) {
        // +1 for pipelining
        ctx->pool = xvbm_buffer_pool_create(xma_plg_get_dev_handle(*
                                            (sk_cmd->xma_session)), payload_data.ibuf_count + 1, payload_data.ibuf_size, 0);
        if (ctx->pool == nullptr) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "%s : xvbm_buffer_pool_create failed\n", __func__);
            goto err5;
        }
        ctx->pool_extended = true;
        ctx->sk_cmd->pool_handle = ctx->pool;
    } else {
        ctx->pool = nullptr;
    }

    sk_cmd->ibuf_ctx->buf_size  = payload_data.ibuf_size;
    sk_cmd->ibuf_ctx->buf_count = payload_data.ibuf_count;


    if (payload_data.qpbuf_size) {
        /* QP */
        sk_cmd->qpbuf_ctx = (buff_pool_ctx_t *)malloc (sizeof (buff_pool_ctx_t));
        if (!(sk_cmd->qpbuf_ctx)) {
            goto err5;
        }

        sk_cmd->qpbuf_ctx->buf_size  = payload_data.qpbuf_size;
        sk_cmd->qpbuf_ctx->buf_count = payload_data.qpbuf_count;
    } else {
        /* continue in non qp mode*/
        sk_cmd->qpbuf_ctx = NULL;
    }

    sk_cmd->obuf_ctx = (buff_pool_ctx_t *)malloc (sizeof (buff_pool_ctx_t));
    if (!(sk_cmd->obuf_ctx)) {
        goto err6;
    }
    sk_cmd->obuf_ctx->buf_size  = payload_data.obuf_size;
    sk_cmd->obuf_ctx->buf_count = payload_data.obuf_count;

    sk_cmd->obuf_ctx->dev_data = (sk_payload_data *) malloc(sizeof (
                                     sk_payload_data));
    if (!sk_cmd->obuf_ctx->dev_data) {
        ret = XMA_ERROR;
        goto err6;
    }

    bzero(sk_cmd->obuf_ctx->dev_data, sizeof(sk_payload_data));
    for (int i = 0; i < MAX_OUT_BUFF_COUNT; i++) {
        sk_cmd->obuf_ctx->dev_data->obuf_info_data[i].obuff_index = 0xBAD;
        sk_cmd->obuf_ctx->dev_data->obuf_info_data[i].recv_size = 0;
        sk_cmd->obuf_ctx->dev_data->obuf_info_data[i].obuf_meta.pts = 0;
    }

    sk_cmd->obuf_ctx->free_index_to_device = 0;
    sk_cmd->obuf_ctx->obuf_free_list = new std::queue <uint32_t>;
    if (sk_cmd->obuf_ctx->obuf_free_list == NULL) {
        ret = XMA_ERROR;
        goto err6;
    }

    /* execute VCU_INIT command */
    ret = prepare_sk_cmd(sk_cmd, VCU_INIT, -1, 0);
    if (ret != XMA_SUCCESS) {
        goto err7;
    }

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "VCU_INIT command failed on device\n");
        goto err8;
    }

    bzero(&payload_data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(xma_session, sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        goto err8;
    }
    memcpy( &payload_data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

    if (!payload_data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "VCU_INIT failed : %s\n",
                   payload_data.dev_err);
        xma_encoder_close(enc_session);
        return XMA_ERROR;
    }

    if (payload_data.extradata_size) {
        uint8_t *app_extradata_buffer = NULL;
        uint32_t app_extradata_buffer_max_size = 0;

        if ((param = getParameter (props->params, props->param_cnt, "extradata"))) {
            app_extradata_buffer = *(uint8_t **)param->value;
            app_extradata_buffer_max_size = param->length;
        }

        if (app_extradata_buffer &&
                payload_data.extradata_size <= app_extradata_buffer_max_size) {
            memcpy(app_extradata_buffer, payload_data.extradata,
                   payload_data.extradata_size);
            if ((param = getParameter (props->params, props->param_cnt,
                                       "extradata_size"))) {
                *((uint32_t *)param->value) = payload_data.extradata_size;
            }
        }
    }

    ctx->m_frame = 0;
    ctx->m_curr_qpbuf_idx = 0;
    ctx->initial_ibufs_consumed = false;
    ctx->frame_sent      = 0;
    ctx->frame_recv      = 0;
    ctx->first_frame     = 0;
    ctx->enc_recv_eos    = false;
    ctx->read_last_frame = false;
    ctx->clear_sb_data   = false;
    ctx->cmd_scheduled   = false;
    ctx->pipeline_widx   = 0;
    ctx->pipeline_ridx   = 0;
    ctx->hdr_frame_count = 0;

    for (int i = 0; i < PIPELINE_DEPTH; ++i) {
        ctx->cmd[i]       = VCU_PREINIT;
        ctx->frame_pts[i] = 0;
    }
    ctx->enc_recv_out_databuf = nullptr;
    ctx->enc_recv_out_data_size = 0;
    ctx->enc_recv_out_data_pts = 0;
    ctx->enc_recv_status = XMA_TRY_AGAIN;
    ctx->enc_recv_free_obuf_idx = -1;

#ifdef MEASURE_TIME
    ctx->send_count = 0;
    ctx->recv_count = 0;
    ctx->send_func_time = 0;
    ctx->recv_func_time = 0;
    ctx->send_xrt_time = 0;
    ctx->recv_xrt_time = 0;
#endif
    ctx->initial_qpbufs_consumed = false;

    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog (LOG_DEBUG, "%s : %p : xma_encoder init finished at %lld \n", __func__,
            ctx, ctx->time_taken);
    try {
        ctx->pending_release =  new std::unordered_multiset<XvbmBufferHandle>;
    } catch(const std::bad_alloc &) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "VCU_INIT Out of host memory\n");
        //@TODO refactor and get rid of goto statements
        goto err8;
    }

    return XMA_SUCCESS;

err8:
    free_bufpool_resources(sk_cmd->obuf_ctx, xma_session);
    sk_cmd->obuf_ctx = nullptr;
    free_bufpool_resources(sk_cmd->qpbuf_ctx, xma_session);
    sk_cmd->qpbuf_ctx = nullptr;
    if (sk_cmd->ibuf_ctx) {
        free (sk_cmd->ibuf_ctx);
    }
    sk_cmd->ibuf_ctx = nullptr;
err7:
    if (sk_cmd->obuf_ctx->obuf_free_list) {
        delete sk_cmd->obuf_ctx->obuf_free_list;
        sk_cmd->obuf_ctx->obuf_free_list = nullptr;
    }
    if (sk_cmd->obuf_ctx->dev_data) {
        free(sk_cmd->obuf_ctx->dev_data);
        sk_cmd->obuf_ctx->dev_data = nullptr;
    }
    free(sk_cmd->obuf_ctx);
    sk_cmd->obuf_ctx = nullptr;
err6:
    if (payload_data.qpbuf_size) {
        free(sk_cmd->qpbuf_ctx);
        sk_cmd->qpbuf_ctx = nullptr;
    }
err5:
    if (ctx->pool) {
        xvbm_buffer_pool_destroy (ctx->pool);
    }
    ctx->pool = NULL;
    if (sk_cmd->ibuf_ctx) {
        free(sk_cmd->ibuf_ctx);
        sk_cmd->ibuf_ctx = nullptr;
    }
err4:
    xma_plg_buffer_free(xma_session, sk_cmd->dynamic_params_buff_obj);
    sk_cmd->dynamic_params_buff = nullptr;
    xma_plg_buffer_free(xma_session, sk_cmd->params_buff_obj);
err3:
    xma_plg_buffer_free(xma_session, sk_cmd->payload_buf_obj);
    return XMA_ERROR;
}

#if DUMP_QP_MAPS
static int32_t dump_frame_delta_qp_map(MpsocEncContext *ctx,
                                       uint8_t *src_qp, uint32_t qp_length)
{
    xrt_cmd_data_t *cmd = ctx->sk_cmd;
    FILE *f_DeltaQpMapHex = NULL;
    uint32_t i = 0;
    char strbuf[512];
    const char *fileName;
    static uint64_t frameNum = 0;

    sprintf(strbuf, "mkdir -p output/enc_%u_%u_%p/", cmd->enc_props->width,
            cmd->enc_props->height, ctx);
    system(strbuf);
    sprintf(strbuf, "output/enc_%u_%u_%p/QP_%lu.hex", cmd->enc_props->width,
            cmd->enc_props->height, ctx, frameNum);
    fileName = strbuf;
    f_DeltaQpMapHex = fopen(fileName, "wb");
    if (NULL == f_DeltaQpMapHex) {
        fprintf(stderr, "*** ERROR: Failed to open file %s\n", fileName);
        return(-1);
    }

    for (i=0; i<qp_length; i++) {
        fprintf(f_DeltaQpMapHex, "%02X\n", src_qp[i]);
    }
    fclose(f_DeltaQpMapHex);
    frameNum++;
    return 0;
}
#endif //#if DUMP_QP_MAPS

#if DUMP_FSFA
typedef struct xlnx_dump_rc_fsfa
{
    uint32_t fs;
    uint32_t fa;
} xlnx_dump_rc_fsfa_t;

static int32_t dump_frame_fsfa(MpsocEncContext *ctx,
                               xlnx_rc_fsfa_t *fsfa, uint32_t fsfa_num)
{
    xrt_cmd_data_t *cmd = ctx->sk_cmd;
    FILE *f_fsfa = NULL;
    char strbuf[512];
    const char *fileName;
    static uint64_t frameNum = 0;
    xlnx_dump_rc_fsfa_t *dump_fsfa = (xlnx_dump_rc_fsfa_t *)fsfa;
    sprintf(strbuf, "mkdir -p output/enc_%u_%u_%p/", cmd->enc_props->width,
            cmd->enc_props->height, ctx);
    system(strbuf);
    sprintf(strbuf, "output/enc_%u_%u_%p/fsfa.txt", cmd->enc_props->width,
            cmd->enc_props->height, ctx);
    fileName = strbuf;
    f_fsfa = fopen(fileName, "a");
    if (NULL == f_fsfa) {
        fprintf(stderr, "*** ERROR: Failed to open file %s\n", fileName);
        return(-1);
    }
    fprintf(f_fsfa, "Frame %lu :\n", frameNum);
    for (uint32_t num=0; num < fsfa_num; num++) {
        fprintf(f_fsfa, "fs[%u]=%u fa[%u]=%u \t", num, dump_fsfa[num].fs, num,
                dump_fsfa[num].fa);
    }
    fprintf(f_fsfa, "\n");

    fclose(f_fsfa);
    frameNum++;
    return 0;
}
#endif //#if DUMP_FSFA

#if DUMP_INPUT_YUV
static int32_t dump_input_yuv(MpsocEncContext *ctx,
                              XvbmBufferHandle xvbmBuf)
{
    if (!xvbmBuf) {
        return 0;
    }
    size_t size = xvbm_buffer_get_size(xvbmBuf);
    uint8_t *hptr = (uint8_t *)xvbm_buffer_get_host_ptr(xvbmBuf);
    if (!hptr || !size) {
        return 0;
    }
    xrt_cmd_data_t *cmd = ctx->sk_cmd;
    FILE *yuv_file = NULL;
    char strbuf[512];
    const char *fileName;

    sprintf(strbuf, "./enc_in_yuv__%u_%u_%p.yuv", cmd->enc_props->width,
            cmd->enc_props->height, ctx);
    fileName = strbuf;
    yuv_file = fopen(fileName, "a");
    if (NULL == yuv_file) {
        fprintf(stderr, "*** ERROR: Failed to open file %s\n", fileName);
        return(-1);
    }

    memset(hptr, 0, size);
    if (xvbm_buffer_read(xvbmBuf, hptr, size, 0)) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "write_dev_buf_to_file: Failed to read device side buffer!!\n");
    } else {
        fwrite(hptr, size, 1, yuv_file);
    }
    fflush(yuv_file);
    fclose(yuv_file);
    return 0;
}
#endif //#if DUMP_INPUT_YUV

static void zero_out_padded_region(uint8_t *dst,
                                   int16_t start_x,
                                   int16_t start_y,
                                   int16_t dst_stride,
                                   int16_t dst_height)
{
    int y, x, idx, line;

    for(y = start_y; y < dst_height; ++y) {
        line = y * dst_stride;
        for(x = start_x; x < dst_stride; ++x) {
            idx = line + x;
            dst[idx] = 0;
        }
    }
}

static int get_raw_host_frame(MpsocEncContext *ctx, XmaFrame *frame,
                              uint32_t widx)
{
    XvbmBufferHandle b_handle = xvbm_buffer_pool_entry_alloc(ctx->pool);
    if(!b_handle) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) Buffer Pool full - no free buffer available\n", __func__);
        return XMA_ERROR;
    }
    uint8_t *src_buffer;
    uint8_t *device_buffer      = (uint8_t *)xvbm_buffer_get_host_ptr(b_handle);
    uint16_t src_bytes_in_line  = frame->frame_props.linesize[0];
    uint16_t dev_bytes_in_line = 0;
    if (frame->frame_props.bits_per_pixel == 10) {
        dev_bytes_in_line  = ALIGN(((frame->frame_props.width+2)/3)*4,
                                   VCU_STRIDE_ALIGN);
    } else {
        dev_bytes_in_line  = ALIGN(frame->frame_props.width, VCU_STRIDE_ALIGN);
    }
    uint16_t src_height         = frame->frame_props.height;
    uint16_t dev_height         = ALIGN(src_height, VCU_HEIGHT_ALIGN);
    size_t dev_y_size           = dev_bytes_in_line * dev_height;
    int ret                     = 0;
    if (src_bytes_in_line != dev_bytes_in_line) {
        uint16_t dev_rows_in_plane = dev_height;
        uint16_t src_rows_in_plane = src_height;
        int16_t stride_delta = dev_bytes_in_line - src_bytes_in_line;
        int16_t height_delta = dev_rows_in_plane - src_rows_in_plane;
        size_t dev_index = 0;
        for (int plane_id = 0; plane_id < xma_frame_planes_get(&frame->frame_props);
                plane_id++) {
            size_t src_index = 0;
            src_buffer = (uint8_t *)frame->data[plane_id].buffer;
            if(plane_id > 0) {
                dev_rows_in_plane = dev_height / 2;
                src_rows_in_plane = src_height / 2;
                height_delta = dev_rows_in_plane - src_rows_in_plane;
            }
            for(uint16_t h = 0; h < src_rows_in_plane && h < dev_rows_in_plane; h++) {
                for(uint16_t w = 0; w < src_bytes_in_line && w < dev_bytes_in_line; w++) {
                    device_buffer[dev_index] = src_buffer[src_index];
                    src_index++;
                    dev_index++;
                }
                if(stride_delta > 0) {
                    dev_index += stride_delta;
                } else {
                    src_index += -1 * stride_delta; // src > dev (higher alignment)
                }
            }
            if(height_delta > 0) {
                dev_index += dev_bytes_in_line * height_delta;
            } // No else necessary because src_index resets.
        }
        ret = xvbm_buffer_write(b_handle, device_buffer,
                                (3 * dev_y_size) >> 1, 0);
    } else {
        size_t src_y_size = src_bytes_in_line * src_height;
        ret = xvbm_buffer_write(b_handle, frame->data[0].buffer, src_y_size, 0);
        if (!ret) {
            ret = xvbm_buffer_write(b_handle, frame->data[1].buffer, src_y_size >> 1,
                                    dev_y_size);
        }
    }
    ctx->sk_cmd->b_handle[widx] = b_handle;
    if (ret) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) DMA to device failed", __func__);
    }
    return ret;
}

static int get_qp_map_data(MpsocEncContext *ctx, XmaFrame *frame)
{
    XmaSideDataHandle sd_handle = xma_frame_get_side_data(frame, XMA_FRAME_QP_MAP);
    int ret = XMA_SUCCESS;

    if (sd_handle && ctx->sk_cmd->qpbuf_ctx) {
        uint8_t *src_qp = (uint8_t *)xma_side_data_get_buffer(sd_handle);
        uint32_t qp_length = xma_side_data_get_size(sd_handle);

#if DUMP_QP_MAPS
        dump_frame_delta_qp_map(ctx, src_qp, qp_length);
#endif //#if DUMP_QP_MAPS
        uint8_t *dev_qp = ctx->sk_cmd->qpbuf_ctx->buf_objs[ctx->m_curr_qpbuf_idx].data;
        uint32_t dev_qp_size = ctx->sk_cmd->qpbuf_ctx->buf_size;
        if (qp_length > (dev_qp_size - 64)) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "Error: Dev qp buffer size (%u) is less than 64 + host qp length (%u)\n",
                       dev_qp_size, qp_length);
            return -1;
        }
        memcpy(dev_qp+64, src_qp, qp_length);
    } else if (frame && (frame->data[0].buffer != NULL) &&
               (sd_handle == NULL) && ctx->sk_cmd->qpbuf_ctx) {
        xma_logmsg(XMA_WARNING_LOG, XMA_VCU_ENCODER,
                   "WARNING : External qpmaps enabled but not supplied to the HW Enc\n");
    }  else if (frame && (frame->data[0].buffer != NULL) &&
                sd_handle && (ctx->sk_cmd->qpbuf_ctx == NULL)) {
        xma_logmsg(XMA_WARNING_LOG, XMA_VCU_ENCODER,
                   "WARNING : HW Enc not configured for external qpmaps\n");
    }

    if (ctx->sk_cmd->qpbuf_ctx) {
        ctx->sk_cmd->qpbuf_idx = ctx->m_curr_qpbuf_idx;
    }

    return ret;
}

static int get_custom_rc_data(MpsocEncContext *ctx, XmaFrame *frame)
{
    XmaSideDataHandle sd_handle = xma_frame_get_side_data(frame, XMA_FRAME_RC_FSFA);

    if (sd_handle) {
        ctx->sk_cmd->fsfa_ptr = (xlnx_rc_fsfa_t *)xma_side_data_get_buffer(sd_handle);
        ctx->sk_cmd->fsfa_num =  xma_side_data_get_size(sd_handle)/sizeof(
                                     xlnx_rc_fsfa_t);
        ctx->sk_cmd->rc_sd_handle = sd_handle;
#if DUMP_FSFA
        dump_frame_fsfa(ctx, ctx->sk_cmd->fsfa_ptr, ctx->sk_cmd->fsfa_num);
#endif //DUMP_FSFA
        /* Supporting a LA Depth of 20 now */
        if (ctx->sk_cmd->fsfa_num != (uint32_t)
                ctx->sk_cmd->enc_props->lookahead_depth) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "%s: ERROR: LA Entries %d does not match LA Depth %d\n", __func__,
                       ctx->sk_cmd->fsfa_num,  ctx->sk_cmd->enc_props->lookahead_depth);
            ctx->sk_cmd->fsfa_num = 0;
            ctx->sk_cmd->fsfa_ptr = nullptr;
            ctx->sk_cmd->rc_sd_handle = nullptr;
            return XMA_ERROR;
        }
        xma_side_data_inc_ref(sd_handle);

    } else {
        /* Lookahead RC Data not present */
        ctx->sk_cmd->fsfa_num = 0;
        ctx->sk_cmd->fsfa_ptr = nullptr;
        ctx->sk_cmd->rc_sd_handle = nullptr;
    }
    return XMA_SUCCESS;
}

static int send_qp_map_data_to_dev(MpsocEncContext *ctx)
{
    XmaBufferObj *qpbuf_objs;
    int ret;

    if (ctx->sk_cmd->qpbuf_ctx) {
        qpbuf_objs = ctx->sk_cmd->qpbuf_ctx->buf_objs;

        ret = xma_plg_buffer_write (*(ctx->sk_cmd->xma_session),
                                    qpbuf_objs[ctx->m_curr_qpbuf_idx], ctx->sk_cmd->qpbuf_ctx->buf_size, 0);
        if (ret != 0) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "xclSyncBO failed %d in %s \n", ret,
                       __func__);
            return XMA_ERROR;
        }
    }
    return XMA_SUCCESS;
}

static void get_dyn_params_data(MpsocEncContext *ctx, XmaFrame *frame)
{
    XmaSideDataHandle sd = xma_frame_get_side_data(frame, XMA_FRAME_DYNAMIC_PARAMS);
    if (sd) {
        enc_dyn_params_t *dyn_param_ptr    = NULL;
        dyn_param_ptr  = (enc_dyn_params_t *)xma_side_data_get_buffer(sd);
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER, "Got enc dyn params in xma plugin %d, %d \n",
                   dyn_param_ptr->bit_rate, dyn_param_ptr->num_b_frames);
        ctx->sk_cmd->obuf_ctx->dev_data->is_dyn_params_valid = 1;
        memcpy(&ctx->sk_cmd->obuf_ctx->dev_data->dyn_params, dyn_param_ptr, sizeof(enc_dyn_params_t));
        xma_frame_remove_side_data_type(frame, XMA_FRAME_DYNAMIC_PARAMS);
    }
    return;
}

static int get_sideband_data(MpsocEncContext *ctx, XmaFrame *frame)
{
    int ret = XMA_ERROR;

    if (XMA_SUCCESS != get_qp_map_data(ctx, frame)) {
        return ret;
    }

    if (XMA_SUCCESS != get_custom_rc_data(ctx, frame)) {
        return ret;
    }

    if(XMA_SUCCESS != send_qp_map_data_to_dev(ctx)) {
        return ret;
    }

    //This routine does not free HDR10 metadata as it is freed by another routine.
    //Make sure HDR data is used before this routine, as this will free all side data.

    get_dyn_params_data(ctx, frame);

    if (ctx->clear_sb_data) {
        xma_frame_clear_all_side_data(frame);
        ctx->clear_sb_data = false;
    }

    return XMA_SUCCESS;
}

static int process_cmd_response(MpsocEncContext *ctx, XmaSession  xma_session)
{
    int ret;
    sk_payload_data payload_data;

    /* check for freed input buffer */
    bzero(&payload_data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(xma_session, ctx->sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) check for free buf failed\n", __func__);
        return ret;
    }
    memcpy(&payload_data, ctx->sk_cmd->payload_buf_obj.data,
           sizeof(sk_payload_data));

    if (!payload_data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) VCU command failed: %s\n", __func__, payload_data.dev_err);
        return XMA_ERROR;
    }

    if (ctx->sk_cmd->qpbuf_ctx) {
        if (ctx->initial_qpbufs_consumed) {
            if (payload_data.freed_qpbuf_index!= 0xBAD) {
                ctx->m_curr_qpbuf_idx = payload_data.freed_qpbuf_index;
            } else {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                           "Error: (%s) qpbuf free: got %x index\n", __func__,
                           payload_data.freed_qpbuf_index);
            }
        } else {
            ctx->m_curr_qpbuf_idx++;
            if (ctx->m_curr_qpbuf_idx == (ctx->sk_cmd->qpbuf_ctx->buf_count - 1)) {
                ctx->initial_qpbufs_consumed = true;
            }
        }
    }

    if ((payload_data.freed_ibuf_index!= 0xBAD) &&
            (payload_data.cmd_id != VCU_FLUSH)) {
        XvbmBufferHandle free_b_handle = xvbm_get_buffer_handle(
                                             ctx->sk_cmd->pool_handle, payload_data.freed_ibuf_index);

        if (free_b_handle) {
            xvbm_buffer_pool_entry_free(free_b_handle);
            XVBM_BUFF_PR("\t\t\tENC: Input buffer consumed. Freeing Input Buffer =%p freed_ibuf_index = %d\n",
                         free_b_handle, payload_data.freed_ibuf_index);
            auto buf_elem = ctx->pending_release->find(free_b_handle);
            if (buf_elem != ctx->pending_release->end()) {
                ctx->pending_release->erase(buf_elem);
            } else {
                xma_logmsg(XMA_WARNING_LOG, XMA_VCU_ENCODER,
                           "%s: b_handle %p already released, something went wrong here\n",
                           __func__, free_b_handle);
            }
        } else {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "%s: b_handle is empty, something went wrong here\n", __func__);
            return XMA_ERROR;
        }
    }
    return XMA_SUCCESS;
}

static int32_t xma_encoder_send_frame(XmaEncoderSession *enc_session,
                                      XmaFrame  *frame)
{
    int ret;

    MpsocEncContext  *ctx  = (MpsocEncContext *) enc_session->base.plugin_data;
    XmaSession  xma_session  = enc_session->base;

    ctx->sk_cmd->xma_session = &xma_session;

#ifdef MEASURE_TIME
    struct timespec start, stop;
    struct timespec rstart, rstop;
    ctx->send_count++;
    clock_gettime (CLOCK_REALTIME, &start);
#endif

    /* donot send frame to encoder kernel, if plugin user doesn't want it to */
    if (frame && frame->do_not_encode && !frame->is_last_frame &&
            (frame->data[0].buffer != NULL)) {
        if (frame->data[0].buffer_type == XMA_DEVICE_BUFFER_TYPE) {
            XvbmBufferHandle ibuf_handle = (XvbmBufferHandle)(frame->data[0].buffer);
            if (ibuf_handle) {
                XVBM_BUFF_PR("ENC : Skiping encoding, Freeing input buffer\n");
                xvbm_buffer_pool_entry_free(ibuf_handle);
            }
        } else {
            frame->data[0].refcount--;
            frame->data[1].refcount--;
            if (frame->data[0].refcount <= 0) {
                for (int32_t i = 0; i < 2 && !frame->data[i].is_clone; i++) {
                    free(frame->data[i].buffer);
                }
                xma_frame_clear_all_side_data(frame);
            }
        }
#ifdef MEASURE_TIME
        clock_gettime (CLOCK_REALTIME, &stop);
        ctx->send_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                (stop.tv_nsec - start.tv_nsec) / 1e3);
        if (ctx->send_count ==  MAX_COUNT_TIME) {
            printf ("Encoder send [%p]: %lld \t %lld\n", ctx,
                    ctx->send_func_time/ctx->send_count, ctx->send_xrt_time/ctx->send_count);
            ctx->send_func_time = ctx->send_xrt_time = ctx->send_count = 0;
        }
#endif
        return XMA_SUCCESS;
    }

    /*----------------------------------
    *   [INPUT]       [Operation]
    *    Raw       DMA Current Frame
    *  Zero Copy   Extract buffer handle
    * ---------------------------------*/
    if (frame && !frame->is_last_frame && (frame->do_not_encode == 0)
            && (frame->data[0].buffer != NULL)) {
        if (ctx->latency_logging) {
            if (ctx->first_frame == 0) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->frame_sent++;
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_enc_first_frame_in %lld : %lld\n", __func__,
                        ctx, ctx->frame_sent, ctx->time_taken);
            }
        }
        if (frame->data[0].buffer_type == XMA_HOST_BUFFER_TYPE) {
            if (get_raw_host_frame(ctx, frame, ctx->pipeline_widx)) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                           "%s: ERROR: host buffer write failed\n", __func__);
                return XMA_ERROR;
            }
            frame->data[0].refcount--;
            frame->data[1].refcount--;
            if (frame->data[0].refcount <= 0) {
                for (int32_t i = 0; i < 2 && !frame->data[i].is_clone; i++) {
                    free(frame->data[i].buffer);
                }

                ctx->clear_sb_data = true; //xma_frame_clear_all_side_data(frame);
            }
        } else {
            ctx->sk_cmd->b_handle[ctx->pipeline_widx] = (XvbmBufferHandle)(
                        frame->data[0].buffer);
#if DUMP_INPUT_YUV
            dump_input_yuv(ctx, (XvbmBufferHandle)(frame->data[0].buffer));
#endif //DUMP_INPUT_YUV

        }
        auto buf_elem = ctx->pending_release->find(ctx->sk_cmd->b_handle[ctx->pipeline_widx]);
        if (buf_elem == ctx->pending_release->end())
            ctx->sk_cmd->duplicate_frame = false;
        else
            ctx->sk_cmd->duplicate_frame = true;
        ctx->pending_release->insert(ctx->sk_cmd->b_handle[ctx->pipeline_widx]);

        if (!ctx->pool_extended) {
            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_extend start at : %lld\n", __func__, ctx,
                        ctx->time_taken);
            }

            uint32_t num = xvbm_buffer_pool_num_buffers_get(
                               ctx->sk_cmd->b_handle[ctx->pipeline_widx]);
            /* TODO: if the incoming pool is shared between two encoder instances,
               pool is extended twice. Need to add a proper check to avoid this.
            */
            uint32_t cnt = xvbm_buffer_pool_extend(
                               ctx->sk_cmd->b_handle[ctx->pipeline_widx], ctx->sk_cmd->ibuf_ctx->buf_count);
            /* Extend by sk_cmd->ibuf_ctx->buf_count number of buffers so that
               total input buffer count >= sk_cmd->ibuf_ctx->buf_count */
            if (cnt == num + ctx->sk_cmd->ibuf_ctx->buf_count) {
                ctx->pool_extended = true;
                xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_ENCODER,
                           "%s : extended earlier output pool to %d buffers\n", __func__, cnt);
            } else {
                ctx->pending_release->erase(ctx->sk_cmd->b_handle[ctx->pipeline_widx]);
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                           "%s : failed to extend memory pool. out of memory ?\n", __func__);
                return XMA_ERROR;
            }

            /* update extended buffers with incoming stride */
            ctx->sk_cmd->stride_width = frame->frame_props.linesize[0];
            ctx->sk_cmd->stride_height = frame->frame_props.linesize[1];
            ctx->sk_cmd->pool_handle = xvbm_get_pool_handle(
                                           ctx->sk_cmd->b_handle[ctx->pipeline_widx]);

            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_extend end at : %lld\n", __func__, ctx,
                        ctx->time_taken);
            }
        }

        /* IDR Insertion */
        ctx->sk_cmd->is_idr[ctx->pipeline_widx] = ((frame &&
                frame->is_idr) ? true : false);
        ctx->frame_pts[ctx->pipeline_widx]      = frame->pts;
        ctx->cmd[ctx->pipeline_widx]            = VCU_PUSH;
        ctx->pipeline_widx                      = (ctx->pipeline_widx + 1) %
                PIPELINE_DEPTH;
    } else {
        ctx->frame_pts[ctx->pipeline_widx]      = frame->pts;
        ctx->cmd[ctx->pipeline_widx]            = VCU_FLUSH;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstart);
#endif

    //Schedule Processing for First Frame
    if(ctx->first_frame < (PIPELINE_DEPTH-1)) {
#ifdef HDR_DATA_SUPPORT
        send_hdr_data_to_sk(enc_session, frame);
#endif
        //extract side-band data from frame
        ret = get_sideband_data(ctx, frame);
        if(ret != XMA_SUCCESS) {
            return XMA_ERROR;
        }

        ret = schedule_cmd(ctx->sk_cmd, ctx->cmd[ctx->pipeline_ridx],
                           ctx->frame_pts[ctx->pipeline_ridx], ctx->pipeline_ridx);
        if (ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "Error: (%s) schedule_cmd failed\n",
                       __func__);
            return XMA_ERROR;
        }
        ctx->cmd_scheduled = true;
        ctx->first_frame++; //need more input frames
        xma_logmsg(XMA_INFO_LOG, XMA_VCU_ENCODER,
                   "Info : (%s) Pipeline Not Full - send more data\n", __func__);
        return XMA_SEND_MORE_DATA;
    }

    /*----------------- Pipeline Active ----------------------*/

    /*-----------------------------
    * Wait For Kerel Completion
    * ----------------------------*/
    if (ctx->cmd_scheduled == true) {
        ctx->cmd_scheduled = false;
        ret = xma_plg_is_work_item_done(xma_session, 5000);
        if(ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "ERROR:: (%s) frame_done signal not received from hw\n", __func__);
            return XMA_ERROR;
        }
    }

    /*-----------------------------
    * Process CMD response
    * ----------------------------*/
    ret = process_cmd_response(ctx, xma_session);
    if(ret != XMA_SUCCESS) {
        return XMA_ERROR;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->frame_sent++;
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog (LOG_DEBUG, "%s : %p : xma_enc_frame_sent %lld : %lld\n", __func__, ctx,
            ctx->frame_sent, ctx->time_taken);
#endif

    /*-----------------------------
    * Read/Recv Frame (from VCU)
    * ----------------------------*/
    if (ctx->enc_recv_out_databuf) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "encoder send(): Last encoded frame not recieved by User!\n");
        return XMA_ERROR;
    }

    ctx->enc_recv_status = run_recv_cmd(ctx, xma_session);

    if (ctx->enc_recv_out_data_size < 0) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "encoder send(): received buffer with zero size, exiting!\n");
        return XMA_ERROR;
    }

    //Check for EOS
    if (((frame->data[0].buffer == NULL) ||
            (ctx->cmd[ctx->pipeline_widx] == VCU_FLUSH)) &&
            (ctx->pipeline_ridx != ctx->pipeline_widx)) {

        //flush all pending buffers (schedule FLUSH cmd)
        ctx->pipeline_ridx = (ctx->pipeline_ridx + 1) % PIPELINE_DEPTH;

        ret = schedule_cmd(ctx->sk_cmd, ctx->cmd[ctx->pipeline_ridx],
                           ctx->frame_pts[ctx->pipeline_ridx], ctx->pipeline_ridx);
        if (ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "Error: (%s) schedule_cmd failed\n",
                       __func__);
            return XMA_ERROR;
        }
        ret = xma_plg_is_work_item_done(xma_session, 5000);
        if(ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "ERROR:: (%s) frame_done signal not received from hw\n", __func__);
            return XMA_ERROR;
        }
        ret = process_cmd_response(ctx, xma_session);
        if(ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "(%s) return XMA_ERROR flush failed\n", __func__);
            return XMA_ERROR;
        }
        ctx->enc_recv_eos = true;
        if (ctx->enc_recv_out_databuf == nullptr) {
            return XMA_FLUSH_AGAIN;
        } else {
            return XMA_SUCCESS;
        }
    }

    //curr frame processed, move to next frame in pipeline
    ctx->pipeline_ridx = (ctx->pipeline_ridx + 1) % PIPELINE_DEPTH;

    /*-----------------------------
    * Schedule NEXT Frame Processing
    * ----------------------------*/
    if ((frame->data[0].buffer != NULL) &&
            ctx->cmd[ctx->pipeline_ridx] == VCU_PUSH) {
#ifdef HDR_DATA_SUPPORT
        send_hdr_data_to_sk(enc_session, frame);
#endif
        //extract side-band data from frame
        ret = get_sideband_data(ctx, frame);
        if(ret != XMA_SUCCESS) {
            return XMA_ERROR;
        }

        ret = schedule_cmd(ctx->sk_cmd, ctx->cmd[ctx->pipeline_ridx],
                           ctx->frame_pts[ctx->pipeline_ridx], ctx->pipeline_ridx);
        if (ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "Error: (%s) schedule_cmd failed\n",
                       __func__);
            return XMA_ERROR;
        }
        ctx->cmd_scheduled = true;
        //measure time
        if (ctx->latency_logging) {
            clock_gettime (CLOCK_REALTIME, &ctx->latency);
            ctx->frame_sent++;
            ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
            syslog (LOG_DEBUG, "%s : %p : xma_enc_frame_sent %lld : %lld\n", __func__, ctx,
                    ctx->frame_sent, ctx->time_taken);
        }
    } else {
        //flush complete
        ctx->enc_recv_eos = true;
    }

    return XMA_SUCCESS;
}

static int32_t run_recv_cmd(MpsocEncContext *ctx, XmaSession  xma_session)
{
    int ret;

    XmaBufferObj *buf_objs = ctx->sk_cmd->obuf_ctx->buf_objs;

#ifdef MEASURE_TIME
    struct timespec start, stop;
    struct timespec rstart, rstop;
    ctx->recv_count++;
    clock_gettime (CLOCK_REALTIME, &start);
#endif

    if(ctx->first_frame < (PIPELINE_DEPTH-1)) {
        return XMA_SUCCESS; //wait for pipeline to fill
    }

    if (ctx->sk_cmd->obuf_ctx->dev_data->freed_index_cnt &&
            ctx->sk_cmd->obuf_ctx->current_free_obuf_index < MAX_OUT_BUFF_COUNT) {
        uint32_t idx =
            ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].obuff_index;
        if (idx != 0xBAD) {
            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->frame_recv++;
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_enc_frame_recv %lld : %lld\n", __func__, ctx,
                        ctx->frame_recv, ctx->time_taken);
            }

            uint32_t size =
                ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].recv_size;

            if (size) {
                ret = xma_plg_buffer_read(xma_session, buf_objs[idx], size, 0);
                if (ret != 0) {
                    /* Ideally the freed out buffer idx should be moved to free list even in failure cases.
                       Not moving it to the free list, to keep the legacy behaviour. */
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "xclSyncBO failed %d in %s \n", ret,
                               __func__);
                    return ret;
                }
                ctx->enc_recv_out_data_size = size;
            } else {
                /* Ideally the freed out buffer idx should be moved to free list even in this case.
                   Not moving it to the free list, to keep the legacy behaviour. */
                ctx->enc_recv_free_obuf_idx = -1;
                ctx->enc_recv_out_data_size = -1;
                ctx->enc_recv_out_databuf = nullptr;
                return XMA_EOS;
            }

            ctx->enc_recv_out_databuf = ctx->sk_cmd->obuf_ctx->buf_objs[idx].data;
            ctx->enc_recv_out_data_pts =
                ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].obuf_meta.pts;
            ctx->enc_recv_free_obuf_idx = idx;
            ctx->sk_cmd->obuf_ctx->dev_data->freed_index_cnt--;
            ctx->sk_cmd->obuf_ctx->current_free_obuf_index++;
#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                printf ("Encoder recv[%p] : %lld \t %lld\n", ctx,
                        ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            return XMA_SUCCESS;
        } else {
#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                printf ("Encoder recv[%p] : %lld \t %lld\n", ctx,
                        ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            syslog (LOG_DEBUG, "Error: %s freed_idx BAD\n", __func__);
            return XMA_TRY_AGAIN;
        }
    }

    /* execute VCU_RECEIVE command */
    ret = prepare_sk_cmd(ctx->sk_cmd, VCU_RECEIVE, -1, 0);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) prepare_sk_cmd(VCU_RECEIVE) failed\n", __func__);
        return XMA_ERROR;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstart);
#endif

    ret = run_xrt_cmd(ctx->sk_cmd);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "Error: (%s) run_xrt_cmd failed\n",
                   __func__);
        return XMA_ERROR;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstop);
    ctx->recv_xrt_time += ((rstop.tv_sec - rstart.tv_sec) * 1e6 +
                           (rstop.tv_nsec - rstart.tv_nsec) / 1e3);
#endif

    ret = xma_plg_buffer_read(xma_session, ctx->sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) xma_plg_buffer_read failed\n", __func__);
        return ret;
    }
    memcpy( ctx->sk_cmd->obuf_ctx->dev_data, ctx->sk_cmd->payload_buf_obj.data,
            sizeof(sk_payload_data));

    if (!ctx->sk_cmd->obuf_ctx->dev_data->cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "Error: (%s) VCU_RECEIVE failed: %s\n", __func__,
                   ctx->sk_cmd->obuf_ctx->dev_data->dev_err);
        return XMA_ERROR;
    }

    if (ctx->sk_cmd->obuf_ctx->dev_data->freed_index_cnt) {
        ctx->sk_cmd->obuf_ctx->current_free_obuf_index = 0;
        uint32_t idx =
            ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].obuff_index;
        if (idx != 0xBAD) {
            uint32_t size =
                ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].recv_size;

            if (size) {
                ret = xma_plg_buffer_read(xma_session, buf_objs[idx], size, 0);
                if (ret != 0) {
                    /* Ideally the freed out buffer idx should be moved to free list even in error case.
                       Not moving it to the free list, to keep the legacy behaviour. */
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "xclSyncBO failed %d in %s \n", ret,
                               __func__);
                    return ret;
                }
                ctx->enc_recv_out_data_size = size;
            } else {
                /* Ideally the freed out buffer idx should be moved to free list even in this case.
                   Not moving it to the free list, to keep the legacy behaviour. */
                ctx->enc_recv_free_obuf_idx = -1;
                ctx->enc_recv_out_data_size = -1;
                ctx->enc_recv_out_databuf = nullptr;
                return XMA_EOS;
            }

            ctx->enc_recv_out_databuf = ctx->sk_cmd->obuf_ctx->buf_objs[idx].data;
            ctx->enc_recv_out_data_pts =
                ctx->sk_cmd->obuf_ctx->dev_data->obuf_info_data[ctx->sk_cmd->obuf_ctx->current_free_obuf_index].obuf_meta.pts;
            ctx->enc_recv_free_obuf_idx = idx;
            ctx->sk_cmd->obuf_ctx->dev_data->freed_index_cnt--;
            ctx->sk_cmd->obuf_ctx->current_free_obuf_index++;

            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->frame_recv++;
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_enc_frame_recv %lld : %lld\n", __func__, ctx,
                        ctx->frame_recv, ctx->time_taken);
                if (ctx->frame_recv == 1) {
                    syslog (LOG_DEBUG, "%s : %p : xma_enc_first_frame_out : %lld\n", __func__, ctx,
                            ctx->time_taken);
                }
            }

#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                printf ("Encoder recv[%p] : %lld \t %lld\n", ctx,
                        ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            return XMA_SUCCESS;
        }
    } else {
        if (ctx->sk_cmd->obuf_ctx->dev_data->end_encoding) {
            return XMA_EOS;
        } else {
#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                printf ("Encoder recv[%p] : %lld \t %lld\n", ctx,
                        ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            syslog (LOG_DEBUG, "Error: (%s) unable to recv data - TRY_AGAIN\n", __func__);
            return XMA_TRY_AGAIN;
        }
    }

    return XMA_TRY_AGAIN;
}


static int32_t xma_encoder_recv(XmaEncoderSession *enc_session,
                                XmaDataBuffer     *recv_data,
                                int32_t           *data_size)
{
    if(recv_data->data.buffer == NULL) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "encoder: output buffer = NULL");
        return XMA_ERROR;
    }
    MpsocEncContext  *ctx  = (MpsocEncContext *) enc_session->base.plugin_data;

    //for last frame (after flush operation), only xma_encoder_recv is called (without xma_encoder_send_frame)
    //"read_last_frame" flag triggers hw read for last frame
    if(ctx->read_last_frame && (ctx->enc_recv_out_databuf == nullptr)) {
        XmaSession  xma_session  = enc_session->base;

        ctx->sk_cmd->xma_session = &xma_session;
        ctx->enc_recv_status = run_recv_cmd(ctx, *ctx->sk_cmd->xma_session);
    }

    if (ctx->enc_recv_out_data_size < 0) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "encoder: received buffer with zero size, exiting!\n");
        *data_size = 0;
        recv_data->is_eof = 1;
        recv_data->data.buffer = NULL;
        return XMA_EOS;
    }

    if (ctx->enc_recv_out_databuf) {
        if (recv_data->alloc_size < ctx->enc_recv_out_data_size) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                       "encoder: output buffer size(%d) < encoded frame size(%d)",
                       recv_data->alloc_size, ctx->enc_recv_out_data_size);
            return XMA_ERROR;
        }
        //recv_data->data.buffer = ctx->enc_recv_out_databuf;
        memcpy(recv_data->data.buffer, ctx->enc_recv_out_databuf,
               ctx->enc_recv_out_data_size);
        *data_size             = ctx->enc_recv_out_data_size;
        recv_data->pts         = ctx->enc_recv_out_data_pts;
        ctx->enc_recv_out_databuf = nullptr;
        ctx->enc_recv_out_data_size = 0;
        ctx->enc_recv_out_data_pts = 0;
        if (ctx->enc_recv_free_obuf_idx >= 0) {
            ctx->sk_cmd->obuf_ctx->obuf_free_list->push(ctx->enc_recv_free_obuf_idx);
            ctx->enc_recv_free_obuf_idx = -1;
        }
    }
    if(ctx->enc_recv_eos) {
        ctx->read_last_frame = true;
    }

    return ctx->enc_recv_status;
}

static int32_t xma_encoder_close(XmaEncoderSession *enc_session)
{
    int ret;
    MpsocEncContext  *ctx  = (MpsocEncContext *) enc_session->base.plugin_data;
    XmaSession  xma_session  = enc_session->base;
    XmaCUCmdObj cu_cmd;

    ctx->frame_sent = 0;
    ctx->frame_recv = 0;

    /* execute VCU_DEINIT command */
    ctx->sk_cmd->xma_session = &xma_session;

    if (ctx->sk_cmd->cu_status == SK_STATUS_BAD) {
        /* if cu status is bad, do not send any further commands(flush/deinit), jump to cleanup */
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "encoder: cu status is bad ! execute 'xbutil reset' to run usecase again\n");
        goto cleanup;
    }

    ret = prepare_sk_cmd(ctx->sk_cmd, VCU_DEINIT, -1, 0);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER, "%s VCU_DEINIT failed\n", __func__);
        return ret;
    }

    cu_cmd = xma_plg_schedule_work_item(*(ctx->sk_cmd->xma_session),
                                        (char *)(ctx->sk_cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);

    while (ret < 0) {
        //sleep(1);
        cu_cmd = xma_plg_schedule_work_item(*(ctx->sk_cmd->xma_session),
                                            (char *)(ctx->sk_cmd->registers), ENC_REGMAP_SIZE*sizeof(uint32_t), &ret);
    };

    ret = xma_plg_is_work_item_done(*(ctx->sk_cmd->xma_session), 5000);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_ENCODER,
                   "*****ERROR:: Encoder stopped responding (%s)*****\n", __func__);
        printf("%s xma_plg_is_work_item_done failed\n", __func__);
        return ret;
    }

cleanup:
    if (ctx->sk_cmd->warn_buff_obj.data) {
        xma_plg_buffer_free (xma_session, ctx->sk_cmd->warn_buff_obj);
    }

    /* free input buffers */
    if (ctx->sk_cmd->ibuf_ctx) {
        free (ctx->sk_cmd->ibuf_ctx);
    }
    ctx->sk_cmd->ibuf_ctx = nullptr;
    /* free output buffers */
    if (ctx->sk_cmd->obuf_ctx) {
        if (ctx->sk_cmd->obuf_ctx->obuf_free_list) {
            delete ctx->sk_cmd->obuf_ctx->obuf_free_list;
            ctx->sk_cmd->obuf_ctx->obuf_free_list = nullptr;
        }
        if (ctx->sk_cmd->obuf_ctx->dev_data) {
            free(ctx->sk_cmd->obuf_ctx->dev_data);
            ctx->sk_cmd->obuf_ctx->dev_data = nullptr;
        }
        free_bufpool_resources(ctx->sk_cmd->obuf_ctx, xma_session);
        ctx->sk_cmd->obuf_ctx = nullptr;
    }

    if (ctx->sk_cmd->qpbuf_ctx) {
        free_bufpool_resources(ctx->sk_cmd->qpbuf_ctx, xma_session);
        ctx->sk_cmd->qpbuf_ctx = nullptr;
    }

    /* free params resource */
    if (ctx->sk_cmd->params_buff) {
        munmap(ctx->sk_cmd->params_buff, ctx->sk_cmd->params_size);
        ctx->sk_cmd->params_buff = nullptr;
    }
    if (ctx->sk_cmd->params_buff_obj.data) {
        xma_plg_buffer_free (xma_session, ctx->sk_cmd->params_buff_obj);
    }

    /* free sk command payload resource */
    if (ctx->sk_cmd->payload_buf_obj.data) {
        xma_plg_buffer_free (xma_session, ctx->sk_cmd->payload_buf_obj);
    }
    if (ctx->sk_cmd->dynamic_params_buff) {
        xma_plg_buffer_free(xma_session, ctx->sk_cmd->dynamic_params_buff_obj);
        ctx->sk_cmd->dynamic_params_buff = nullptr;
    }

    /* free sk command resource */
    free_xrt_res(ctx->sk_cmd);
    ctx->sk_cmd = nullptr;

    if (ctx->pending_release) {
        for (auto buf : (*ctx->pending_release)) {
            xvbm_buffer_pool_entry_free(buf);
        }
        ctx->pending_release->clear();
        delete ctx->pending_release;
        ctx->pending_release = nullptr;
    }
    if (ctx->pool) {
        xvbm_buffer_pool_destroy(ctx->pool);
        ctx->pool = NULL;
    }

    closelog();
    return XMA_SUCCESS;
}

static int32_t xma_encoder_version(int32_t *main_version, int32_t *sub_version)
{
    *main_version = 2020;
    *sub_version = 1;

    return 0;
}


XmaEncoderPlugin encoder_plugin = {
    .hwencoder_type = XMA_MULTI_ENCODER_TYPE,
    .hwvendor_string = "MPSoC",
    .format = XMA_VCU_NV12_FMT_TYPE,
    .bits_per_pixel = 8,
    .plugin_data_size = sizeof(MpsocEncContext),
    .init = xma_encoder_init,
    .send_frame = xma_encoder_send_frame,
    .recv_data = xma_encoder_recv,
    .close = xma_encoder_close,
    .xma_version    = xma_encoder_version
};

