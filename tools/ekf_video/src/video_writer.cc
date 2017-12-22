/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ekf_video/video_writer.h>

#include <QtGui/QImage>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <assert.h>

namespace ekf_video {

bool libav_initialized = false;

void InitializeLibAv(void) {
  if (libav_initialized)
    return;
  libav_initialized = true;

  av_register_all();
  avcodec_register_all();
}

VideoWriter::VideoWriter(const char* videofile, int width, int height) :
     width_(width), height_(height) {
  InitializeLibAv();
  OpenVideo(videofile);
}

VideoWriter::~VideoWriter(void) {
  CloseVideo();
}

void VideoWriter::OpenVideo(const char* videofile) {
  // open container
  avformat_alloc_output_context2(&format_context_, NULL, NULL, videofile);
  if (format_context_ == NULL) {
    fprintf(stderr, "Could not allocate format context.\n");
    exit(1);
  }
  assert(format_context_->oformat->video_codec != AV_CODEC_ID_NONE);

  // open codec
  AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (codec == NULL) {
    fprintf(stderr, "Could not open encoder.\n");
    exit(1);
  }
  assert(codec->type == AVMEDIA_TYPE_VIDEO);
  AVStream* stream = avformat_new_stream(format_context_, codec);
  if (stream == NULL) {
    fprintf(stderr, "Could not open stream.\n");
    exit(1);
  }
  stream->id = 0;
  stream->time_base.num = 1;
  stream->time_base.den = 15;
  codec_context_ = stream->codec;

  avcodec_get_context_defaults3(codec_context_, codec);
  codec_context_->codec_id = AV_CODEC_ID_H264;
  codec_context_->bit_rate = 400000;
  codec_context_->width  = width_;
  codec_context_->height = height_;
  codec_context_->gop_size = 15;
  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 15;
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  AVDictionary* dict = NULL;
  av_dict_set(&dict, "preset", "veryslow", 0);
  av_dict_set_int(&dict, "crf", 10, 0);
  int result = avcodec_open2(codec_context_, codec, &dict);
  if (result < 0) {
    fprintf(stderr, "Failed to open codec with error %d.\n", result);
    exit(1);
  }
  av_dict_free(&dict);

  // allocate frame
  frame_ = av_frame_alloc();
  if (frame_ == NULL) {
    fprintf(stderr, "Failed to allocate frame.\n");
    exit(1);
  }
  frame_->format = codec_context_->pix_fmt;
  frame_->width = codec_context_->width;
  frame_->height = codec_context_->height;
  result = av_frame_get_buffer(frame_, 32);
  if (result < 0) {
    fprintf(stderr, "Failed to allocate picture.\n");
    exit(1);
  }

  av_dump_format(format_context_, 0, videofile, 1);

  if (!(format_context_->oformat->flags & AVFMT_NOFILE)) {
    result = avio_open(&format_context_->pb, videofile, AVIO_FLAG_WRITE);
    if (result < 0) {
      fprintf(stderr, "Failed to open file %s with error %d.\n", videofile, result);
      exit(1);
    }
  }
  result = avformat_write_header(format_context_, NULL);
  if (result < 0) {
    fprintf(stderr, "Failed to write header.\n");
    exit(1);
  }

  frame_->pts = 0;
}

void VideoWriter::CloseVideo(void) {
  // flush encoder
  int output_received = 0;
  do {
    int result = EncodeFrame(NULL, &output_received);
    if (result < 0) {
      fprintf(stderr, "Error flushing codec.\n");
      break;
    }
  } while (output_received);

  // close everything
  av_write_trailer(format_context_);
  for (unsigned int i = 0; i < format_context_->nb_streams; i++)
    if (format_context_->streams[i]->codec)
      avcodec_close(format_context_->streams[i]->codec);
  av_frame_free(&frame_);
  if (!(format_context_->oformat->flags & AVFMT_NOFILE))
    avio_close(format_context_->pb);
  avformat_free_context(format_context_);
  format_context_ = NULL;
}

int VideoWriter::EncodeFrame(AVFrame* frame, int* output_received) {
  AVPacket packet;
  av_init_packet(&packet);
  packet.data = NULL;
  packet.size = 0;

  int update = 0;
  int result = avcodec_encode_video2(codec_context_, &packet, frame, &update);
  if (output_received)
    *output_received = update;
  if (result < 0) {
    fprintf(stderr, "Failed to encode frame with error %d.\n", result);
    return result;
  }
  if (update) {
    if (frame && frame->key_frame)
      packet.flags |= AV_PKT_FLAG_KEY;
    packet.pts = av_rescale_q_rnd(packet.pts, codec_context_->time_base,
                                  format_context_->streams[0]->time_base, AV_ROUND_NEAR_INF);
    packet.dts = av_rescale_q_rnd(packet.dts, codec_context_->time_base,
                                  format_context_->streams[0]->time_base, AV_ROUND_NEAR_INF);
    packet.duration = av_rescale_q(packet.duration, codec_context_->time_base, format_context_->streams[0]->time_base);
    packet.stream_index = 0;
    result = av_interleaved_write_frame(format_context_, &packet);
    if (result < 0) {
      fprintf(stderr, "Failed to write frame with error %d.\n", result);
      return result;
    }
  }
  av_free_packet(&packet);

  return 0;
}

void VideoWriter::AddFrame(const QImage & image) {
  struct SwsContext* context = sws_getContext(image.width(), image.height(), AV_PIX_FMT_BGRA,
                               width_, height_, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);
  int result = av_frame_make_writable(frame_);
  if (result < 0) {
    fprintf(stderr, "Failed to make frame writable.\n");
    exit(1);
  }

  const unsigned char* source_image = image.constBits();
  int image_size = image.bytesPerLine();

  sws_scale(context, &source_image, &image_size, 0, image.height(), frame_->data, frame_->linesize);
  sws_freeContext(context);

  if (frame_->pts % 15 == 0)
    frame_->key_frame = 1;
  else
    frame_->key_frame = 0;

  result = EncodeFrame(frame_, NULL);
  if (result < 0) {
    fprintf(stderr, "Failed to encode frame.\n");
    exit(1);
  }

  frame_->pts++;
}

}  // namespace ekf_video

