/*
 * Copyright (C) 2019, Xilinx Inc - All rights reserved
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
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <syslog.h>
#include <unordered_set>

#include <xmaplugin.h>

#undef MEASURE_TIME
#ifdef MEASURE_TIME
#define MAX_COUNT_TIME 1000
#include <time.h>
#endif

#define WIDTH_ALIGN             256
#define HEIGHT_ALIGN            64
#define ALIGN(size,align)       (((size) + (align) - 1) & ~((align) - 1))

/* FRM_BUF_POOL_SIZE value comes from vcu control software which is 50 as of 2019.2 release.
   This  value is used in corresponding decoder softkernel
*/
#define FRM_BUF_POOL_SIZE       50
#define DEC_REGMAP_SIZE         20

#include <xvbm.h>
#include <list>
/* #define XVBM_BUFF_PR(...) printf(__VA_ARGS__) */
#define XVBM_BUFF_PR(...)

#define MAX_ERR_STRING  1024
#define XMA_VCU_DECODER "xma-vcu-decoder"

/* defines from control software */
#define MAX_STACK_SIZE 16

/* Limits */
#define H264_DEC_MIN_WIDTH     80
#define H264_DEC_MIN_HEIGHT    96
#define H264_DEC_MAX_WIDTH    3840
#define H264_DEC_MAX_HEIGHT   2160

#define HEVC_DEC_MIN_WIDTH     128
#define HEVC_DEC_MIN_HEIGHT    128
#define HEVC_DEC_MAX_WIDTH    3840
#define HEVC_DEC_MAX_HEIGHT   2160

#define MULTI_DEC_MIN_WIDTH    128
#define MULTI_DEC_MIN_HEIGHT   128
#define MULTI_DEC_MAX_WIDTH   3840
#define MULTI_DEC_MAX_HEIGHT  2160

#define AVC_PROFILE_IDC_BASELINE 66
#define AVC_PROFILE_IDC_MAIN     77
#define AVC_PROFILE_IDC_HIGH     100
#define HEVC_PROFILE_IDC_MAIN    1
static int32_t xma_decoder_close(XmaDecoderSession *dec_session);

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

typedef struct dec_params
{
    uint32_t bitdepth;
    uint32_t codec_type;
    uint32_t low_latency;
    uint32_t entropy_buffers_count;
    uint32_t frame_rate;
    uint32_t clk_ratio;
    uint32_t profile;
    uint32_t level;
    uint32_t height;
    uint32_t width;
    uint32_t chroma_mode;
    uint32_t scan_type;
    uint32_t splitbuff_mode;
} dec_params_t;

typedef struct _vcu_dec_usermeta
{
    int64_t pts;
} vcu_dec_usermeta;

typedef struct _out_buf_info
{
    uint64_t freed_obuf_paddr;
    size_t freed_obuf_size;
    uint32_t freed_obuf_index;
} out_buf_info;

#define MAX_OUT_INFOS 25

typedef struct host_dev_data
{
    uint32_t cmd_id;
    uint32_t cmd_rsp;
    uint32_t obuff_size;
    uint32_t obuff_num;
    uint32_t obuff_index[FRM_BUF_POOL_SIZE];
    uint32_t ibuff_valid_size;
    uint32_t host_to_dev_ibuf_idx;
    uint32_t dev_to_host_ibuf_idx;
    bool last_ibuf_copied;
    bool resolution_found;
    vcu_dec_usermeta ibuff_meta;
    vcu_dec_usermeta obuff_meta[FRM_BUF_POOL_SIZE];
    bool end_decoding;
    uint32_t free_index_cnt;
    int valid_oidxs;
    out_buf_info obuf_info[MAX_OUT_INFOS];
    char dev_err[MAX_ERR_STRING];
} sk_payload_data;

#define MAX_IBUFFS 2

typedef struct buff_pool_ctx
{
    XmaBufferObj buff_obj;
    XmaBufferObj buff_obj_arr[MAX_IBUFFS];
//  uint64_t    buff_paddr;
    uint32_t    buf_size;
} buff_pool_ctx_t;

typedef struct obuff_pool_ctx
{
    XmaBufferObj parent_buff_obj;
    uint64_t    parent_buff_paddr;
    XmaBufferObj *buf_objs;
    uint32_t    buf_size;
    uint32_t    buf_count;
    XvbmPoolHandle out_pool_handle;
    // sk does not return unused buffers, when flushed
    std::unordered_multiset<XvbmBufferHandle> *pending_release;
    sk_payload_data *dev_data;
    uint32_t current_free_obuf_index;
} obuff_pool_ctx_t;

typedef struct xrt_cmd_data
{
    uint32_t           registers[DEC_REGMAP_SIZE];
    size_t        size;
    XmaBufferObj   payload_buf_obj;
    uint64_t      payload_buf_paddr;
    XmaSession    *xma_session;
    buff_pool_ctx_t   *ibuf_ctx;
    obuff_pool_ctx_t  *obuf_ctx;
    buff_pool_ctx_t   *paramsbuf_ctx;
    dec_params_t    params_data;
    uint32_t ibuff_valid_size;
    XmaDecoderSession *dec_session;
    XvbmBufferHandle o_xvbm_buffer_handle;
    uint32_t use_zero_copy;
    uint32_t host_to_dev_ibuf_idx;
    dev_cu_status cu_status;
    uint64_t timestamp;
} xrt_cmd_data_t;

typedef struct MpsocDecContext
{
    xrt_cmd_data_t  *sk_cmd;
    bool obuff_allocated;
    size_t luma_bufsize;
    size_t chroma_bufsize;
    bool end_sent;
    bool sk_error;
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
    uint8_t *last_buf_ptr;
} MpsocDecContext;

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

    if (!cmd || cmd->cu_status == SK_STATUS_BAD) {
        return XMA_ERROR;
    }

    XmaCUCmdObj cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                         (char *)(cmd->registers), DEC_REGMAP_SIZE*sizeof(uint32_t), &ret);

    while (ret < 0) {
        //sleep(1);
        cu_cmd = xma_plg_schedule_work_item(*(cmd->xma_session),
                                            (char *)(cmd->registers), DEC_REGMAP_SIZE*sizeof(uint32_t), &ret);
    };

    ret = xma_plg_is_work_item_done(*(cmd->xma_session), 5000);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "*****ERROR:: Decoder stopped responding (%s)*****\n", __func__);
        cmd->cu_status = SK_STATUS_BAD;
        return ret;
    }

    return XMA_SUCCESS;
}

static void free_xrt_res(xrt_cmd_data *cmd)
{
    if (cmd) {
        free(cmd);
    }
}

static void
fill_free_outbuffer_indexes (xrt_cmd_data_t *cmd,  sk_payload_data *data)
{
    int free_pool_size;
    int i;
    uint64_t paddr;

    if (!cmd->obuf_ctx || !(cmd->obuf_ctx->out_pool_handle)) {
        return;
    }

    free_pool_size = xvbm_get_freelist_count(cmd->obuf_ctx->out_pool_handle);

    // reset all obuf indexes
    for (i = 0; i < MAX_OUT_INFOS; i++) {
        data->obuf_info[i].freed_obuf_index = 0xBAD;
    }

    if (free_pool_size > MAX_OUT_INFOS) {
        xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                   "free pool size is greater than max out infos array...restricting to max out infos\n");
        free_pool_size = MAX_OUT_INFOS;
    }
    data->valid_oidxs = free_pool_size;

    for (i = 0; i < free_pool_size; i++) {
        cmd->o_xvbm_buffer_handle = xvbm_buffer_pool_entry_alloc(
                                        cmd->obuf_ctx->out_pool_handle);
        if (cmd->o_xvbm_buffer_handle) {
            cmd->obuf_ctx->pending_release->insert(cmd->o_xvbm_buffer_handle);
            cmd->obuf_ctx->buf_count = xvbm_buffer_pool_num_buffers_get(
                                           cmd->o_xvbm_buffer_handle);
            paddr = xvbm_buffer_get_paddr(cmd->o_xvbm_buffer_handle);
            if (!paddr) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                           "%s : Error - no free output buffer available\n", __func__);
                return;
            }

            data->obuf_info[i].freed_obuf_index = xvbm_buffer_get_id(
                    cmd->o_xvbm_buffer_handle);
            data->obuf_info[i].freed_obuf_paddr = paddr;
            data->obuf_info[i].freed_obuf_size = xvbm_buffer_get_size(
                    cmd->o_xvbm_buffer_handle);
        } else {
            data->obuf_info[i].freed_obuf_index = 0xBAD;
            data->obuf_info[i].freed_obuf_paddr = 0;
            data->obuf_info[i].freed_obuf_size = 0;
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                       "Error: (%s) obuf_ctx Buffer Pool full - no free buffer available\n", __func__);
        }
    }
}

static bool validate_params_data(dec_params_t *data)
{
    /* limit checks in line with FFMpeg plugin */

    if (data->bitdepth != 8) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "bitdepth %d is not supported\n",
                   data->bitdepth);
        return false;
    }

    if (data->codec_type > 1) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "Invalid codec type specified. Valid: 0 or 1\n");
        return false;
    }

    if (data->low_latency > 1) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "Invalid low latency flag. Valid: 0 or 1\n");
        return false;
    }

    if (data->splitbuff_mode > 1) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "Invalid split buffer mode flag. Valid: 0 or 1\n");
        return false;
    }

    if (data->entropy_buffers_count < 2 || data->entropy_buffers_count > 10) {
        xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                   "invalid entropy_buffers_count. Valid range: [2..10], setting default to 2\n");
        data->entropy_buffers_count = 2;
    }

    if (data->chroma_mode != 420) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "chroma mode %d is not supported\n",
                   data->chroma_mode);
        return false;
    }

    if (data->codec_type == 0) {
        /* H264 */
        if ((data->profile != AVC_PROFILE_IDC_BASELINE) &&
                (data->profile != AVC_PROFILE_IDC_MAIN) &&
                (data->profile != AVC_PROFILE_IDC_HIGH)) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "profile %d is not supported\n",
                       data->profile);
            return false;
        }

        if (!data->level) {
            data->level = 62;
        }

        switch (data->level) {
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 20:
            case 21:
            case 22:
            case 30:
            case 31:
            case 32:
            case 40:
            case 41:
            case 42:
            case 50:
            case 51:
            case 52:
            /* 6.2 is found to be supported, though spec says it isn't */
            case 62:
                break;
            default:
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "level %d is not supported\n",
                           data->level);
                return false;
        }
    } else {
        /* HEVC */
        if (data->profile != HEVC_PROFILE_IDC_MAIN) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "profile %d is not supported\n",
                       data->profile);
            return false;
        }

        if (!data->level) {
            data->level = 186;
        }

        /* As per spec., HEVC general_level_idc = 30 * level. control sw needs 10 * level.
         * Ex: For 4.1 level, general_level_idc = data->level = 123. control sw needs its equivalent as 41 (123/3) */
        data->level = data->level/3;
        switch (data->level) {
            case 10:
            case 20:
            case 21:
            case 30:
            case 31:
            case 40:
            case 41:
            case 50:
            case 51:
            /* 6.2 is found to be supported, though spec says it isn't */
            case 62:
                break;
            default:
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "level %d is not supported\n",
                           data->level);
                return false;
        }
    }

    return true;
}

static int prepare_sk_cmd(xrt_cmd_data_t *cmd, enum cmd_type type, int64_t pts)
{
    int ret;
    sk_payload_data data;
    XmaParameter *param;
    XmaDecoderProperties *props;
    int idx = 0;

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
    cmd->registers[++idx]  = cmd->payload_buf_paddr & 0xFFFFFFFF;
    cmd->registers[++idx]  = ((uint64_t)(cmd->payload_buf_paddr) >> 32) &
                             0xFFFFFFFF;
    cmd->registers[++idx]  = sizeof(sk_payload_data);


    data.cmd_id = type;
    switch (type) {
        case VCU_PREINIT:
            cmd->paramsbuf_ctx->buf_size = sizeof(dec_params_t);
            cmd->paramsbuf_ctx->buff_obj = xma_plg_buffer_alloc(*(cmd->xma_session),
                                           cmd->paramsbuf_ctx->buf_size, false, &ret);
            if (ret != XMA_SUCCESS) {
                ret = XMA_ERROR;
                break;
            }

            cmd->registers[++idx] = cmd->paramsbuf_ctx->buff_obj.paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->paramsbuf_ctx->buff_obj.paddr) >> 32) &
                                    0xFFFFFFFF;
            cmd->registers[++idx] = cmd->paramsbuf_ctx->buf_size;

            /* fill the decoder params */
            props = &cmd->dec_session->decoder_props;

            if ((param = getParameter (props->params, props->param_cnt, "bitdepth"))) {
                cmd->params_data.bitdepth = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "codec_type"))) {
                cmd->params_data.codec_type = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "low_latency"))) {
                cmd->params_data.low_latency = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt,
                                       "entropy_buffers_count"))) {
                cmd->params_data.entropy_buffers_count = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "zero_copy"))) {
                cmd->use_zero_copy = *(uint32_t *)param->value;
            } else {
                cmd->use_zero_copy = 1;
            }
            if ((param = getParameter (props->params, props->param_cnt, "profile"))) {
                cmd->params_data.profile = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "level"))) {
                cmd->params_data.level = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "chroma_mode"))) {
                cmd->params_data.chroma_mode = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt, "scan_type"))) {
                cmd->params_data.scan_type = *(uint32_t *)param->value;
            }
            if ((param = getParameter (props->params, props->param_cnt,
                                       "splitbuff_mode"))) {
                cmd->params_data.splitbuff_mode = *(uint32_t *)param->value;
            }

            /* validate and assign default values if not present */
            if (!validate_params_data(&cmd->params_data)) {
                xma_plg_buffer_free(*(cmd->xma_session), cmd->paramsbuf_ctx->buff_obj);
                ret = XMA_ERROR;
                break;
            }

            cmd->params_data.height = props->height;
            cmd->params_data.width =  props->width;
            cmd->params_data.frame_rate = props->framerate.numerator;
            cmd->params_data.clk_ratio = props->framerate.denominator;

            memcpy ( cmd->paramsbuf_ctx->buff_obj.data, &(cmd->params_data),
                     cmd->paramsbuf_ctx->buf_size);
            ret = xma_plg_buffer_write (*(cmd->xma_session), cmd->paramsbuf_ctx->buff_obj,
                                        cmd->paramsbuf_ctx->buf_size, 0);

            if (ret != XMA_SUCCESS) {
                xma_plg_buffer_free(*(cmd->xma_session), cmd->paramsbuf_ctx->buff_obj);
                ret = XMA_ERROR;
                break;
            }
            break;

        case VCU_INIT:
            if (!cmd->ibuf_ctx->buf_size) {
                ret = XMA_ERROR;
                break;
            }

            for (int i = 0; i < MAX_IBUFFS; i++) {
                cmd->ibuf_ctx->buff_obj_arr[i] = xma_plg_buffer_alloc(*(cmd->xma_session),
                                                 cmd->ibuf_ctx->buf_size, false, &ret);
                if (ret != XMA_SUCCESS) {
                    ret = XMA_ERROR;
                    goto err1;
                }

                cmd->registers[++idx] = cmd->ibuf_ctx->buff_obj_arr[i].paddr & 0xFFFFFFFF;
                cmd->registers[++idx] = ((uint64_t)(cmd->ibuf_ctx->buff_obj_arr[i].paddr) >> 32)
                                        & 0xFFFFFFFF;
                cmd->registers[++idx] = cmd->ibuf_ctx->buf_size;
            }

            if (!cmd->obuf_ctx->buf_count || !cmd->obuf_ctx->buf_size) {
                ret = XMA_ERROR;
                goto err1;
            }

            data.obuff_size = cmd->obuf_ctx->buf_size;
            data.obuff_num = cmd->obuf_ctx->buf_count;
            uint32_t i;
            uint32_t offset;
            uint64_t paddr;
            XvbmBufferHandle o_handle;

            cmd->obuf_ctx->dev_data = (sk_payload_data *) malloc(sizeof (sk_payload_data));
            if (!cmd->obuf_ctx->dev_data) {
                ret = XMA_ERROR;
                goto err1;
            }

            bzero(cmd->obuf_ctx->dev_data, sizeof(sk_payload_data));
            for (i = 0; i < FRM_BUF_POOL_SIZE; i++) {
                cmd->obuf_ctx->dev_data->obuff_index[i] = 0xBAD;
                cmd->obuf_ctx->dev_data->obuff_meta[i].pts = 0;
            }
            try {
                cmd->obuf_ctx->pending_release =  new std::unordered_multiset<XvbmBufferHandle>;
            } catch(const std::bad_alloc &) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "%s Out of host memory\n", __func__);
                //@TODO refactor and get rid of goto statements
                ret = XMA_ERROR;
                goto err2;
            }

//      cmd->obuf_ctx->out_pool_handle = xvbm_buffer_pool_create_by_device_id(cmd->xma_session->hw_session.dev_index, data.obuff_num, data.obuff_size, 0);
            cmd->obuf_ctx->out_pool_handle = xvbm_buffer_pool_create(xma_plg_get_dev_handle(
                                                 *(cmd->xma_session)), data.obuff_num, data.obuff_size, 0);
            if (cmd->obuf_ctx->out_pool_handle == nullptr) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "Decoder pool creation failed\n");
                ret = XMA_ERROR;
                goto err2;
            }

            cmd->obuf_ctx->parent_buff_obj = xma_plg_buffer_alloc(*(cmd->xma_session),
                                             cmd->obuf_ctx->buf_count * sizeof (uint64_t), false, &ret);
            if(ret != XMA_SUCCESS) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                           "parent_buff_handle failed for outbuf\n");
                ret = XMA_ERROR;
                goto err3;
            }
            cmd->obuf_ctx->parent_buff_paddr = cmd->obuf_ctx->parent_buff_obj.paddr;
            if (!cmd->obuf_ctx->parent_buff_paddr) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                           "parent_buff_paddr for outbuf failed\n");
                ret = XMA_ERROR;
                goto err4;
            }

            offset = 0;
            for (i = 0; i < data.obuff_num; i++) {
                o_handle = xvbm_buffer_pool_entry_alloc(cmd->obuf_ctx->out_pool_handle);
                if (o_handle) {
                    cmd->obuf_ctx->pending_release->insert(o_handle);
                    paddr = xvbm_buffer_get_paddr(o_handle);

                    memcpy ( cmd->obuf_ctx->parent_buff_obj.data+offset, &paddr, sizeof (uint64_t));
                    ret = xma_plg_buffer_write (*(cmd->xma_session), cmd->obuf_ctx->parent_buff_obj,
                                                sizeof (uint64_t), offset);
                    if (ret != XMA_SUCCESS) {
                        ret = XMA_ERROR;
                        goto err4;
                    }

                    offset += sizeof (uint64_t);
                } else {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "cant get output handle\n");
                    ret = XMA_ERROR;
                    goto err4;
                }
            }
            /* TODO: check below indxes*/
            cmd->registers[++idx] = cmd->obuf_ctx->parent_buff_paddr & 0xFFFFFFFF;
            cmd->registers[++idx] = ((uint64_t)(cmd->obuf_ctx->parent_buff_paddr) >> 32) &
                                    0xFFFFFFFF;
            cmd->registers[++idx] = cmd->obuf_ctx->buf_count * sizeof (uint64_t);


            break;
err4:
            xma_plg_buffer_free(*(cmd->xma_session), cmd->obuf_ctx->parent_buff_obj);
err3:
            xvbm_buffer_pool_destroy(cmd->obuf_ctx->out_pool_handle);
err2:
            if (cmd->obuf_ctx->pending_release) {
                delete cmd->obuf_ctx->pending_release;
                cmd->obuf_ctx->pending_release = nullptr;
            }
            if (cmd->obuf_ctx->dev_data) {
                free (cmd->obuf_ctx->dev_data);
                cmd->obuf_ctx->dev_data = NULL;
            }
err1:
            for (int i = 0; i < MAX_IBUFFS; i++) {
                if (cmd->ibuf_ctx->buff_obj_arr[i].data) {
                    xma_plg_buffer_free(*(cmd->xma_session), cmd->ibuf_ctx->buff_obj_arr[i]);
                }
            }
            break;

        case VCU_PUSH:
            fill_free_outbuffer_indexes (cmd, &data);

            data.host_to_dev_ibuf_idx = cmd->host_to_dev_ibuf_idx;
            data.ibuff_valid_size = cmd->ibuff_valid_size;
            data.ibuff_meta.pts = pts;

            break;
        case VCU_RECEIVE:
            fill_free_outbuffer_indexes (cmd, &data);
            break;
        case VCU_DEINIT:
            break;
        case VCU_FLUSH:
            /* do nothing */
            usleep(30000);// 30msec sleep added for NULL frame used during flush
            break;
        default:
            ret = XMA_ERROR;
            break;
    }

    if (ret == XMA_SUCCESS) {
        memcpy ( cmd->payload_buf_obj.data, &data, sizeof(sk_payload_data));
        ret = xma_plg_buffer_write (*(cmd->xma_session), cmd->payload_buf_obj,
                                    sizeof(sk_payload_data), 0);
    }

    return ret;
}

static int32_t xma_decoder_init(XmaDecoderSession *dec_session)
{
    int ret;
    sk_payload_data data;
    struct timespec init_time;

    if(*(uint8_t *)dec_session->base.plugin_data != 0) {
        return XMA_ERROR;
    }

    openlog ("XMA_Decoder", LOG_PID, LOG_USER);

    MpsocDecContext  *ctx  = (MpsocDecContext *) dec_session->base.plugin_data;
    XmaSession  xma_session  = dec_session->base;
    const XmaDecoderProperties *props = (XmaDecoderProperties *)(&
                                        (dec_session->decoder_props));
    //Based of https://www.xilinx.com/support/documentation/sw_manuals/xilinx2020_2/pg252-vcu.pdf
    // and taking into consideration potrait and landscape orientations
    if (!(props->width % 2 || props->height % 2)) {
        if(props->hwdecoder_type == XMA_H264_DECODER_TYPE) {
            if(!(props->width >= H264_DEC_MIN_WIDTH  &&
                    props->height >= H264_DEC_MIN_HEIGHT)   ||
                    !(props->width >= H264_DEC_MIN_HEIGHT &&
                      props->height >= H264_DEC_MIN_WIDTH)    ||
                    !(props->width * props->height >= H264_DEC_MIN_WIDTH * H264_DEC_MIN_HEIGHT)) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                           "error :  minimum resolution not met for H264 Decoder  : %d x %d \n",
                           props->width, props->height);
                return XMA_ERROR;
            } else if(!(props->width <= H264_DEC_MAX_WIDTH  &&
                        props->height <= H264_DEC_MAX_HEIGHT)   ||
                      !(props->width <= H264_DEC_MAX_HEIGHT &&
                        props->height <= H264_DEC_MAX_WIDTH)   ||
                      !(props->width * props->height <= H264_DEC_MAX_WIDTH * H264_DEC_MAX_HEIGHT)) {
                xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                           "error :  maximum resolution exceeded for H264 Decoder  : %d x %d \n",
                           props->width, props->height);
                return XMA_ERROR;
            }
        } else if (props->hwdecoder_type == XMA_HEVC_DECODER_TYPE) {
            if(!(props->width >= HEVC_DEC_MIN_WIDTH  &&
                    props->height >= HEVC_DEC_MIN_HEIGHT)   ||
                    !(props->width >= HEVC_DEC_MIN_HEIGHT && props->height >= HEVC_DEC_MIN_WIDTH)) {
                if(props->width * props->height >= HEVC_DEC_MIN_WIDTH * HEVC_DEC_MIN_HEIGHT) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "error :  minimum resolution not met for HEVC Decoder  : %d x %d \n",
                               props->width, props->height);
                    return XMA_ERROR;
                }
            } else if(!(props->width <= HEVC_DEC_MAX_WIDTH  &&
                        props->height <= HEVC_DEC_MAX_HEIGHT)   ||
                      !(props->width <= HEVC_DEC_MAX_HEIGHT && props->height <= HEVC_DEC_MAX_WIDTH)) {
                if(props->width * props->height <= HEVC_DEC_MAX_WIDTH * HEVC_DEC_MAX_HEIGHT) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "error :  maximum resolution exceeded for HEVC Decoder  : %d x %d \n",
                               props->width, props->height);
                    return XMA_ERROR;
                }
            }
        } else if (props->hwdecoder_type == XMA_MULTI_DECODER_TYPE) {
            if(!(props->width >= MULTI_DEC_MIN_WIDTH  &&
                    props->height >= MULTI_DEC_MIN_HEIGHT)   ||
                    !(props->width >= MULTI_DEC_MIN_HEIGHT &&
                      props->height >= MULTI_DEC_MIN_WIDTH)) {
                if(props->width * props->height < MULTI_DEC_MIN_WIDTH * MULTI_DEC_MIN_HEIGHT) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "error :  minimum resolution not met for MULTI Decoder  : %d x %d \n",
                               props->width, props->height);
                    return XMA_ERROR;
                }
            } else if(!(props->width <= MULTI_DEC_MAX_WIDTH  &&
                        props->height <= MULTI_DEC_MAX_HEIGHT)   ||
                      !(props->width <= MULTI_DEC_MAX_HEIGHT &&
                        props->height <= MULTI_DEC_MAX_WIDTH)) {
                if(props->width * props->height > MULTI_DEC_MAX_WIDTH * MULTI_DEC_MAX_HEIGHT) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "error :  maximum resolution exceeded for MULTI Decoder  : %d x %d \n",
                               props->width, props->height);
                    return XMA_ERROR;
                }
            }
        }
    } else {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "error : resolution should be even : %d x %d \n", props->width, props->height);
        return XMA_ERROR;
    }

    ctx->luma_bufsize = (ALIGN(props->width, WIDTH_ALIGN) * ALIGN(props->height,
                         HEIGHT_ALIGN));
    ctx->chroma_bufsize = ctx->luma_bufsize >> 1;
    ctx->sk_error = false;
    ctx->end_sent = true;

    syslog (LOG_DEBUG, "xma_dec_handle = %p\n", ctx);
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog (LOG_DEBUG, "%s : %p : xma_decoder init start  at %lld\n", __func__, ctx,
            ctx->time_taken);

    /* configure soft kernel - command structure */
    xrt_cmd_data_t *sk_cmd = (xrt_cmd_data *) calloc(1, sizeof(xrt_cmd_data_t));
    if (!sk_cmd) {
        return XMA_ERROR;
    }

    sk_cmd->dec_session = dec_session;

    sk_cmd->size = 1024 * sizeof(int);
    sk_cmd->payload_buf_obj = xma_plg_buffer_alloc (xma_session,
                              sizeof(sk_payload_data), false, &ret);
    if (ret != XMA_SUCCESS) {
        free(sk_cmd);
        return XMA_ERROR;
    }

    sk_cmd->payload_buf_paddr =  sk_cmd->payload_buf_obj.paddr;
    sk_cmd->xma_session = &xma_session;
    sk_cmd->cu_status = SK_STATUS_OKAY;

    ctx->obuff_allocated = false;
    sk_cmd->paramsbuf_ctx = (buff_pool_ctx_t *)malloc (sizeof (buff_pool_ctx_t));
    if (!(sk_cmd->paramsbuf_ctx)) {
        goto err3;
    }

    clock_gettime (CLOCK_MONOTONIC, &init_time);
    sk_cmd->timestamp = ((init_time.tv_sec * 1e6) + (init_time.tv_nsec/1e3));

    ret = prepare_sk_cmd(sk_cmd, VCU_PREINIT, -1);
    if (ret != XMA_SUCCESS) {
        goto err4;
    }

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        goto err5;
    }

    bzero(&data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(*(sk_cmd->xma_session), sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        goto err5;
    }
    memcpy(&data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

    if (!data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "VCU_PREINIT failed: %s\n",
                   data.dev_err);
        goto err5;
    }

    sk_cmd->obuf_ctx = (obuff_pool_ctx_t *)malloc (sizeof (obuff_pool_ctx_t));
    if (!(sk_cmd->obuf_ctx)) {
        goto err5;
    }

    sk_cmd->obuf_ctx->buf_size  = data.obuff_size;
    sk_cmd->obuf_ctx->buf_count = data.obuff_num;

    sk_cmd->ibuf_ctx = (buff_pool_ctx_t *)malloc (sizeof (buff_pool_ctx_t));
    if (!(sk_cmd->ibuf_ctx)) {
        goto err6;
    }

    /* allocate uncompressed size to accomodate compressed size, which should be sufficient */
    sk_cmd->ibuf_ctx->buf_size  = (props->width * props->height);

    ctx->sk_cmd = sk_cmd;

    ret = prepare_sk_cmd(sk_cmd, VCU_INIT, -1);
    if (ret != XMA_SUCCESS) {
        goto err7;
    }

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        goto err7;
    }

    bzero(&data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(*(sk_cmd->xma_session), sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        goto err7;
    }
    memcpy(&data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

    if (!data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "VCU_INIT failed: %s\n",
                   data.dev_err);
        xma_decoder_close(dec_session);
        return XMA_ERROR;
    }

#ifdef MEASURE_TIME
    ctx->send_count = 0;
    ctx->recv_count = 0;
    ctx->send_func_time = 0;
    ctx->recv_func_time = 0;
    ctx->send_xrt_time = 0;
    ctx->recv_xrt_time = 0;
#endif

    ctx->frame_sent = 0;
    ctx->frame_recv = 0;
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog (LOG_DEBUG, "%s : %p : xma_decoder init done  at %lld\n", __func__, ctx,
            ctx->time_taken);

    ctx->obuff_allocated = true;
    sk_cmd->host_to_dev_ibuf_idx = 0;
    ctx->last_buf_ptr = NULL;

    xma_plg_buffer_free(xma_session, sk_cmd->paramsbuf_ctx->buff_obj);
    free(sk_cmd->paramsbuf_ctx);
    sk_cmd->paramsbuf_ctx = NULL;

    XmaParameter *param;
    if ((param = getParameter (props->params, props->param_cnt,
                               "latency_logging"))) {
        ctx->latency_logging = *(uint32_t *)param->value;
    } else {
        ctx->latency_logging = 0;
    }

    return XMA_SUCCESS;
err7:
    for (int i = 0; i < MAX_IBUFFS; i++) {
        if (sk_cmd->ibuf_ctx->buff_obj_arr[i].data)
          xma_plg_buffer_free(xma_session, sk_cmd->ibuf_ctx->buff_obj_arr[i]);
    }
    free(sk_cmd->ibuf_ctx);
    sk_cmd->ibuf_ctx = NULL;
err6:
    free(sk_cmd->obuf_ctx);
    sk_cmd->obuf_ctx = NULL;
err5:
    xma_plg_buffer_free(xma_session, sk_cmd->paramsbuf_ctx->buff_obj);
err4:
    free(sk_cmd->paramsbuf_ctx);
    sk_cmd->paramsbuf_ctx = NULL;

err3:
    xma_plg_buffer_free(xma_session, sk_cmd->payload_buf_obj);
    return XMA_ERROR;
}

static int32_t xma_decoder_send(XmaDecoderSession *dec_session,
                                XmaDataBuffer *data,
                                int32_t *data_used)
{
    int ret;
    sk_payload_data payload_data;

    MpsocDecContext  *ctx  = (MpsocDecContext *) dec_session->base.plugin_data;
    XmaSession  xma_session  = dec_session->base;
    xrt_cmd_data_t *sk_cmd = ctx->sk_cmd;
    ctx->sk_cmd->xma_session = &xma_session;
#ifdef MEASURE_TIME
    struct timespec start, stop;
    struct timespec rstart, rstop;
    ctx->send_count++;
    clock_gettime (CLOCK_REALTIME, &start);
#endif
    /* XMA_ERROR in this callback(send_data) is handled by
       'close' callback called via application */
    if (data->is_eof) {
        ctx->end_sent = true;

        /* execute VCU_FLUSH command */
        ret = prepare_sk_cmd(sk_cmd, VCU_FLUSH, -1);
        if (ret != XMA_SUCCESS) {
            return XMA_ERROR;
        }

        ret = run_xrt_cmd(sk_cmd);
        if (ret != XMA_SUCCESS) {
            return ret;
        }

        bzero(&payload_data, sizeof(sk_payload_data));
        ret = xma_plg_buffer_read(*(sk_cmd->xma_session), sk_cmd->payload_buf_obj,
                                  sizeof(sk_payload_data), 0);
        if (ret != XMA_SUCCESS) {
            return ret;
        }
        memcpy(&payload_data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

        if (!payload_data.cmd_rsp) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "[%s][%d] VCU_FLUSH failed: %s\n",
                       __func__, __LINE__, payload_data.dev_err);
            return XMA_ERROR;
        }

        return XMA_SUCCESS;
    }
    if (ctx->sk_error) {
        return XMA_ERROR;
    }

    sk_cmd->ibuff_valid_size = data->alloc_size;

    ret = prepare_sk_cmd(sk_cmd, VCU_PUSH, data->pts);
    if (ret != XMA_SUCCESS) {
        return XMA_ERROR;
    }

    if (data->alloc_size) {
        memcpy (sk_cmd->ibuf_ctx->buff_obj_arr[sk_cmd->host_to_dev_ibuf_idx].data,
                data->data.buffer, data->alloc_size);
        ret = xma_plg_buffer_write (xma_session,
                                    sk_cmd->ibuf_ctx->buff_obj_arr[sk_cmd->host_to_dev_ibuf_idx], data->alloc_size,
                                    0);
        if (ret != XMA_SUCCESS) {
            xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                       "[%s][%d]xma_dec : failed to do write...\n", __func__, __LINE__);
            return ret;
        }
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstart);
#endif

    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        ctx->sk_error = true;
        return ret;
    }
#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstop);
    ctx->send_xrt_time += ((rstop.tv_sec - rstart.tv_sec) * 1e6 +
                           (rstop.tv_nsec - rstart.tv_nsec) / 1e3);
#endif
    /* all the data will be copied on device side, so mark total size as used */
    *data_used = data->alloc_size;

    /* check for request for output buffer allocation */
    bzero(&payload_data, sizeof(sk_payload_data));
    ret = xma_plg_buffer_read(xma_session, sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "[%s][%d]xma_dec : failed to do read...\n", __func__, __LINE__);
        return ret;
    }
    memcpy(&payload_data, sk_cmd->payload_buf_obj.data, sizeof(sk_payload_data));

    if (!payload_data.cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "[%s][%d] VCU_PUSH failed: %s\n",
                   __func__, __LINE__, payload_data.dev_err);
        return XMA_ERROR;
    }

    if (ctx->latency_logging) {
        clock_gettime (CLOCK_REALTIME, &ctx->latency);
        ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
        syslog (LOG_DEBUG, "%s : %p : xma_send done  at %lld\n", __func__, ctx,
                ctx->time_taken);
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &stop);
    ctx->send_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                            (stop.tv_nsec - start.tv_nsec) / 1e3);
    if (ctx->send_count ==  MAX_COUNT_TIME) {
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "\nDecoder send : %lld \t %lld\n",
                   ctx->send_func_time/ctx->send_count, ctx->send_xrt_time/ctx->send_count);
        ctx->send_func_time = ctx->send_xrt_time = ctx->send_count = 0;
    }
#endif

    /* zero data size (as flag) is only to push free output buffer indexes. so always success */
    if (!data->alloc_size) {
        return XMA_SUCCESS;
    }

    if (payload_data.dev_to_host_ibuf_idx != 0xBAD) {
        sk_cmd->host_to_dev_ibuf_idx =  payload_data.dev_to_host_ibuf_idx;

        /* for latency, log the 'decoder sent timestamp' only when device consumed input */
        ctx->frame_sent++;
        if (ctx->latency_logging) {
            syslog(LOG_DEBUG, "%s : %p : xma_dec_frame_sent %lld : %lld\n", __func__, ctx,
                   ctx->frame_sent, ctx->time_taken);
        }
        return XMA_SUCCESS;
    } else {
        return XMA_TRY_AGAIN;
    }
}

static int32_t xma_decoder_recv(XmaDecoderSession *dec_session, XmaFrame *frame)
{
    int ret;

    MpsocDecContext  *ctx  = (MpsocDecContext *) dec_session->base.plugin_data;
    XmaSession  xma_session  = dec_session->base;
    xrt_cmd_data_t *sk_cmd = ctx->sk_cmd;
#ifdef MEASURE_TIME
    struct timespec start, stop;
    struct timespec rstart, rstop;
    ctx->recv_count++;
    clock_gettime (CLOCK_REALTIME, &start);
#endif

    if (ctx->sk_error) {
        return XMA_ERROR;
    }

    if (!ctx->obuff_allocated) {
        xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "%s: No out buff allocated yet\n",
                   __func__);
#ifdef MEASURE_TIME
        clock_gettime (CLOCK_REALTIME, &stop);
        ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                (stop.tv_nsec - start.tv_nsec) / 1e3);
        if (ctx->recv_count ==  MAX_COUNT_TIME) {
            xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "Decoder recv : %lld \t %lld\n",
                       ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
            ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
        }
#endif
        return XMA_TRY_AGAIN;
    }

    ctx->sk_cmd->xma_session = &xma_session;

    if (sk_cmd->obuf_ctx->dev_data->free_index_cnt &&
	sk_cmd->obuf_ctx->current_free_obuf_index < FRM_BUF_POOL_SIZE) {
        uint32_t idx =
            sk_cmd->obuf_ctx->dev_data->obuff_index[sk_cmd->obuf_ctx->current_free_obuf_index];
        if (idx != 0xBAD) {
            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->frame_recv++;
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_dec_frame_recv %lld : %lld\n", __func__, ctx,
                        ctx->frame_recv, ctx->time_taken);
            }

            if (sk_cmd->use_zero_copy) {
                frame->data[0].buffer = (XvbmBufferHandle) xvbm_get_buffer_handle(
                                            sk_cmd->obuf_ctx->out_pool_handle, idx);
                if (sk_cmd->obuf_ctx->pending_release->erase(frame->data[0].buffer) == 0) {
                    xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                               "%s: b_handle(%p) already released?, something went wrong here\n",
                               __func__, frame->data[0].buffer);
                }
            } else {
                uint32_t size = sk_cmd->obuf_ctx->buf_size * 2 / 3;
                uint8_t *hbuf;
                int32_t ret;
                XvbmBufferHandle b_handle;

                b_handle = xvbm_get_buffer_handle(sk_cmd->obuf_ctx->out_pool_handle, idx);
                if (!b_handle) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "[%s][%d]xma_dec : invalid idx\n",
                               __func__, __LINE__);
                    return XMA_ERROR;
                }
                hbuf     = (uint8_t *)xvbm_buffer_get_host_ptr(b_handle);
                if (!hbuf) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "[%s][%d]xma_dec : invalid host buffer\n", __func__, __LINE__);
                    return XMA_ERROR;
                }
                ret      = xvbm_buffer_read(b_handle, hbuf, sk_cmd->obuf_ctx->buf_size, 0);
                if (ret) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "[%s][%d]xma_dec : failed to do read...\n", __func__, __LINE__);
                    return XMA_ERROR;
                }
                memcpy (frame->data[0].buffer, hbuf, size);
                memcpy (frame->data[1].buffer, hbuf + size, size / 2);
                xvbm_buffer_pool_entry_free (xvbm_get_buffer_handle(
                                                 sk_cmd->obuf_ctx->out_pool_handle, idx));
                if (sk_cmd->obuf_ctx->pending_release->erase(b_handle) == 0) {
                    xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                               "%s: b_handle(%p) already released?, something went wrong here\n",
                               __func__, b_handle);
                }
            }
            frame->pts =
                sk_cmd->obuf_ctx->dev_data->obuff_meta[sk_cmd->obuf_ctx->current_free_obuf_index].pts;
            sk_cmd->obuf_ctx->dev_data->free_index_cnt--;
            sk_cmd->obuf_ctx->current_free_obuf_index++;
#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "Decoder recv : %lld \t %lld\n",
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
                xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "Decoder recv : %lld \t %lld\n",
                           ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            if (sk_cmd->obuf_ctx->dev_data->end_decoding) {
                return XMA_EOS;
            }
        }
    }

    ret = prepare_sk_cmd(sk_cmd, VCU_RECEIVE, -1);
    if (ret != XMA_SUCCESS) {
        return ret;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstart);
#endif

    ret = run_xrt_cmd(sk_cmd);
    if (ret != XMA_SUCCESS) {
        ctx->sk_error = true;
        return ret;
    }

#ifdef MEASURE_TIME
    clock_gettime (CLOCK_REALTIME, &rstop);
    ctx->recv_xrt_time += ((rstop.tv_sec - rstart.tv_sec) * 1e6 +
                           (rstop.tv_nsec - rstart.tv_nsec) / 1e3);
#endif

    ret = xma_plg_buffer_read(xma_session, sk_cmd->payload_buf_obj,
                              sizeof(sk_payload_data), 0);
    if (ret != XMA_SUCCESS) {
        return ret;
    }
    memcpy( sk_cmd->obuf_ctx->dev_data, sk_cmd->payload_buf_obj.data,
            sizeof(sk_payload_data));

    if (!sk_cmd->obuf_ctx->dev_data->cmd_rsp) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "[%s][%d] VCU_RECEIVE failed: %s\n",
                   __func__, __LINE__, sk_cmd->obuf_ctx->dev_data->dev_err);
        return XMA_ERROR;
    }

    if (sk_cmd->obuf_ctx->dev_data->free_index_cnt) {
        sk_cmd->obuf_ctx->current_free_obuf_index = 0;
        uint32_t idx =
            sk_cmd->obuf_ctx->dev_data->obuff_index[sk_cmd->obuf_ctx->current_free_obuf_index];
        if (idx != 0xBAD) {
            if (ctx->latency_logging) {
                clock_gettime (CLOCK_REALTIME, &ctx->latency);
                ctx->frame_recv++;
                ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
                syslog (LOG_DEBUG, "%s : %p : xma_dec_frame_recv %lld : %lld\n", __func__, ctx,
                        ctx->frame_recv, ctx->time_taken);
            }

            if (sk_cmd->use_zero_copy) {
                frame->data[0].buffer = (XvbmBufferHandle) xvbm_get_buffer_handle(
                                            sk_cmd->obuf_ctx->out_pool_handle, idx);
                if (sk_cmd->obuf_ctx->pending_release->erase(frame->data[0].buffer) == 0) {
                    xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                               "%s: b_handle(%p) already released?, something went wrong here\n",
                               __func__, frame->data[0].buffer);
                }
            } else {
                uint32_t size = sk_cmd->obuf_ctx->buf_size * 2 / 3;
                uint8_t *hbuf;
                int32_t ret;
                XvbmBufferHandle b_handle;

                b_handle = xvbm_get_buffer_handle(sk_cmd->obuf_ctx->out_pool_handle, idx);
                if (!b_handle) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER, "[%s][%d]xma_dec : invalid idx\n",
                               __func__, __LINE__);
                    return XMA_ERROR;
                }

                hbuf     = (uint8_t *)xvbm_buffer_get_host_ptr(b_handle);
                if (!hbuf) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "[%s][%d]xma_dec : invalid host buffer\n", __func__, __LINE__);
                    return XMA_ERROR;
                }
                ret      = xvbm_buffer_read(b_handle, hbuf, sk_cmd->obuf_ctx->buf_size, 0);
                if (ret) {
                    xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                               "[%s][%d]xma_dec : failed to do read...\n", __func__, __LINE__);
                    return XMA_ERROR;
                }
                memcpy (frame->data[0].buffer, hbuf, size);
                memcpy (frame->data[1].buffer, hbuf + size, size / 2);
                xvbm_buffer_pool_entry_free (xvbm_get_buffer_handle(
                                                 sk_cmd->obuf_ctx->out_pool_handle, idx));
                if (sk_cmd->obuf_ctx->pending_release->erase(b_handle) == 0) {
                    xma_logmsg(XMA_WARNING_LOG, XMA_VCU_DECODER,
                               "%s: b_handle(%p) already released?, something went wrong here\n",
                               __func__, b_handle);
                }
            }
            frame->pts =
                sk_cmd->obuf_ctx->dev_data->obuff_meta[sk_cmd->obuf_ctx->current_free_obuf_index].pts;
        }
        sk_cmd->obuf_ctx->dev_data->free_index_cnt--;
        sk_cmd->obuf_ctx->current_free_obuf_index++;
#ifdef MEASURE_TIME
        clock_gettime (CLOCK_REALTIME, &stop);
        ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                (stop.tv_nsec - start.tv_nsec) / 1e3);
        if (ctx->recv_count ==  MAX_COUNT_TIME) {
            xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "Decoder recv : %lld \t %lld\n",
                       ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
            ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
        }
#endif
        return XMA_SUCCESS;
    } else {
        if (sk_cmd->obuf_ctx->dev_data->end_decoding) {
            return XMA_EOS;
        } else {
#ifdef MEASURE_TIME
            clock_gettime (CLOCK_REALTIME, &stop);
            ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 +
                                    (stop.tv_nsec - start.tv_nsec) / 1e3);
            if (ctx->recv_count ==  MAX_COUNT_TIME) {
                xma_logmsg(XMA_DEBUG_LOG, XMA_VCU_DECODER, "Decoder recv : %lld \t %lld\n",
                           ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
                ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
            }
#endif
            return XMA_TRY_AGAIN;
        }
    }
}

static int32_t xma_decoder_close(XmaDecoderSession *dec_session)
{
    int ret;
    MpsocDecContext  *ctx  = (MpsocDecContext *) dec_session->base.plugin_data;
    XmaSession  xma_session  = dec_session->base;
    xrt_cmd_data_t *sk_cmd = ctx->sk_cmd;
    XmaCUCmdObj cu_cmd;

    if (!sk_cmd) {
        return XMA_ERROR;
    }

    ctx->sk_cmd->xma_session = &xma_session;

    ctx->frame_sent = 0;
    ctx->frame_recv = 0;

    if (sk_cmd->cu_status == SK_STATUS_BAD) {
        /* if cu status is bad, do not send any further commands(flush/deinit), jump to cleanup */
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "decoder: cu status is bad ! execute 'xbutil reset' to run usecase again\n");
        goto cleanup;
    }

    if (!ctx->end_sent) {
        XmaDataBuffer buf;
        buf.is_eof = true;
        int32_t used = 0;
        xma_decoder_send (dec_session, &buf, &used);
    }

    ctx->sk_cmd->xma_session = &xma_session;
    ret = prepare_sk_cmd(ctx->sk_cmd, VCU_DEINIT, -1);
    if (ret != XMA_SUCCESS) {
        return ret;
    }

    cu_cmd = xma_plg_schedule_work_item(*(sk_cmd->xma_session),
                                        (char *)(sk_cmd->registers), DEC_REGMAP_SIZE*sizeof(uint32_t), &ret);

    while (ret < 0) {
        //sleep(1);
        cu_cmd = xma_plg_schedule_work_item(*(sk_cmd->xma_session),
                                            (char *)(sk_cmd->registers), DEC_REGMAP_SIZE*sizeof(uint32_t), &ret);
    };

    ret = xma_plg_is_work_item_done(*(sk_cmd->xma_session), 5000);
    if (ret != XMA_SUCCESS) {
        xma_logmsg(XMA_ERROR_LOG, XMA_VCU_DECODER,
                   "*****ERROR:: Decoder stopped responding (%s)*****\n", __func__);
        return ret;
    }

cleanup:

    /* free decoder params */
    if (sk_cmd->paramsbuf_ctx) {
        if (sk_cmd->paramsbuf_ctx->buff_obj.data) {
            xma_plg_buffer_free (xma_session, sk_cmd->paramsbuf_ctx->buff_obj);
        }

        free(sk_cmd->paramsbuf_ctx);
    }

    for (int i=0; i < MAX_IBUFFS; i++) {
        /* free ibuff resoure */
        if (sk_cmd->ibuf_ctx->buff_obj_arr[i].data) {
            xma_plg_buffer_free (xma_session, sk_cmd->ibuf_ctx->buff_obj_arr[i]);
        }
    }

    if (sk_cmd->ibuf_ctx) {
        free(sk_cmd->ibuf_ctx);
    }

    /* free obuff resoure */
    if (ctx->obuff_allocated) {
        ctx->obuff_allocated = false;
        if (sk_cmd->obuf_ctx->pending_release) {
            for (auto buf : (*sk_cmd->obuf_ctx->pending_release)) {
                xvbm_buffer_pool_entry_free(buf);
            }
            sk_cmd->obuf_ctx->pending_release->clear();
            delete sk_cmd->obuf_ctx->pending_release;
            sk_cmd->obuf_ctx->pending_release = nullptr;
        }
        if (sk_cmd->obuf_ctx->dev_data) {
            free(sk_cmd->obuf_ctx->dev_data);
        }
        xvbm_buffer_pool_destroy(sk_cmd->obuf_ctx->out_pool_handle);
        xma_plg_buffer_free(xma_session, sk_cmd->obuf_ctx->parent_buff_obj);
        free(sk_cmd->obuf_ctx);
    }

    /* free sk command payload resource */
    if (sk_cmd->payload_buf_obj.data) {
        xma_plg_buffer_free (xma_session, sk_cmd->payload_buf_obj);
    }

    free_xrt_res(sk_cmd);

    closelog();

    return XMA_SUCCESS;
}

static int32_t xma_decoder_version(int32_t *main_version, int32_t *sub_version)
{
    *main_version = 2020;
    *sub_version = 1;

    return 0;
}

static int32_t xma_decoder_get_properties (XmaDecoderSession *dec_session,
        XmaFrameProperties *fprops)
{
    const XmaDecoderProperties *props = (XmaDecoderProperties *)(&
                                        (dec_session->decoder_props));
    MpsocDecContext  *ctx  = (MpsocDecContext *) dec_session->base.plugin_data;
    xrt_cmd_data_t *sk_cmd = ctx->sk_cmd;

    if (fprops) {
        fprops->format = XMA_VCU_NV12_FMT_TYPE;
        fprops->width = props->width;
        fprops->height = props->height;
        fprops->bits_per_pixel = sk_cmd->params_data.bitdepth;
        return XMA_SUCCESS;
    } else {
        return XMA_ERROR;
    }
}

XmaDecoderPlugin decoder_plugin = {
    .hwdecoder_type = XMA_MULTI_DECODER_TYPE,
    .hwvendor_string = "MPSoC",
    .plugin_data_size = sizeof(MpsocDecContext),
    .init = xma_decoder_init,
    .send_data = xma_decoder_send,
    .get_properties   = xma_decoder_get_properties,
    .recv_frame = xma_decoder_recv,
    .close = xma_decoder_close,
    .xma_version    = xma_decoder_version
};
