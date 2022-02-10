#!/usr/bin/env python3

################################################################################
# SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import sys

sys.path.append('../')
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst, GstRtspServer
import sys
import os
from optparse import OptionParser
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.utils import long_to_uint64
from common.FPS import GETFPS
import pyds
import math
import configparser

MAX_DISPLAY_LEN = 64
MAX_TIME_STAMP_LEN = 32
PGIE_CLASS_ID_DoorWarningSign = 0
PGIE_CLASS_ID_People = 1
PGIE_CLASS_ID_TwoWheeler = 2
# PGIE_CLASS_ID_ROADSIGN = 3
MUXER_OUTPUT_WIDTH = 1920
MUXER_OUTPUT_HEIGHT = 1080
MUXER_BATCH_TIMEOUT_USEC = 4000000
TILED_OUTPUT_WIDTH = 1280
TILED_OUTPUT_HEIGHT = 720
input_file = None
schema_type = 0
proto_lib = None
conn_str = "localhost;2181;testTopic"
cfg_file = None
input_src_uri = None
topic = None
sensor_id_str = None
no_display = False
no_output_rtsp = False
pgie_config_file = ""

MSCONV_CONFIG_FILE = "dstest51_msgconv_config.txt"

pgie_classes_str = ["DoorWarningSign", "People", "TwoWheeler", "____", "Roadsign"]
fps_streams = {}


# tiler_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
# and update params for drawing rectangle, object information etc.
def tiler_src_pad_buffer_probe(pad, info, u_data):
    frame_number = 0
    num_rects = 0
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number = frame_meta.frame_num
        l_obj = frame_meta.obj_meta_list
        num_rects = frame_meta.num_obj_meta
        obj_counter = {
            PGIE_CLASS_ID_DoorWarningSign: 0,
            PGIE_CLASS_ID_TwoWheeler: 0,
            PGIE_CLASS_ID_People: 0,
            # PGIE_CLASS_ID_ROADSIGN: 0
        }
        while l_obj is not None:
            try:
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
                if (frame_number % 30) == 0:
                    # Frequency of messages to be send will be based on use case.
                    # Here message is being sent for first object every 30 frames.

                    # Allocating an NvDsEventMsgMeta instance and getting
                    # reference to it. The underlying memory is not manged by
                    # Python so that downstream plugins can access it. Otherwise
                    # the garbage collector will free it when this probe exits.
                    msg_meta = pyds.alloc_nvds_event_msg_meta()
                    msg_meta.bbox.top = obj_meta.rect_params.top
                    msg_meta.bbox.left = obj_meta.rect_params.left
                    msg_meta.bbox.width = obj_meta.rect_params.width
                    msg_meta.bbox.height = obj_meta.rect_params.height
                    msg_meta.frameId = frame_number
                    msg_meta.trackingId = long_to_uint64(obj_meta.object_id)
                    msg_meta.confidence = obj_meta.confidence
                    msg_meta = generate_event_msg_meta(msg_meta, obj_meta.class_id)
                    user_event_meta = pyds.nvds_acquire_user_meta_from_pool(
                        batch_meta)
                    if user_event_meta:
                        user_event_meta.user_meta_data = msg_meta
                        user_event_meta.base_meta.meta_type = pyds.NvDsMetaType.NVDS_EVENT_MSG_META
                        # Setting callbacks in the event msg meta. The bindings
                        # layer will wrap these callables in C functions.
                        # Currently only one set of callbacks is supported.
                        pyds.user_copyfunc(user_event_meta, meta_copy_func)
                        pyds.user_releasefunc(user_event_meta, meta_free_func)
                        pyds.nvds_add_user_meta_to_frame(frame_meta,
                                                         user_event_meta)
                    else:
                        print("Error in attaching event meta to buffer\n")
            except StopIteration:
                break
            obj_counter[obj_meta.class_id] += 1
            try:
                l_obj = l_obj.next
            except StopIteration:
                break
        print("Frame Number=", frame_number, "Number of Objects=", num_rects, "DoorWarningSign_count=",
              obj_counter[PGIE_CLASS_ID_DoorWarningSign], "TwoWheeler_count=", obj_counter[PGIE_CLASS_ID_TwoWheeler])

        # Get frame rate through this probe
        fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()
        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK


# Callback function for deep-copying an NvDsEventMsgMeta struct
def meta_copy_func(data, user_data):
    # Cast data to pyds.NvDsUserMeta
    user_meta = pyds.NvDsUserMeta.cast(data)
    src_meta_data = user_meta.user_meta_data
    # Cast src_meta_data to pyds.NvDsEventMsgMeta
    srcmeta = pyds.NvDsEventMsgMeta.cast(src_meta_data)
    # Duplicate the memory contents of srcmeta to dstmeta
    # First use pyds.get_ptr() to get the C address of srcmeta, then
    # use pyds.memdup() to allocate dstmeta and copy srcmeta into it.
    # pyds.memdup returns C address of the allocated duplicate.
    dstmeta_ptr = pyds.memdup(pyds.get_ptr(srcmeta),
                              sys.getsizeof(pyds.NvDsEventMsgMeta))
    # Cast the duplicated memory to pyds.NvDsEventMsgMeta
    dstmeta = pyds.NvDsEventMsgMeta.cast(dstmeta_ptr)

    # Duplicate contents of ts field. Note that reading srcmeat.ts
    # returns its C address. This allows to memory operations to be
    # performed on it.
    dstmeta.ts = pyds.memdup(srcmeta.ts, MAX_TIME_STAMP_LEN + 1)

    # Copy the sensorStr. This field is a string property. The getter (read)
    # returns its C address. The setter (write) takes string as input,
    # allocates a string buffer and copies the input string into it.
    # pyds.get_string() takes C address of a string and returns the reference
    # to a string object and the assignment inside the binder copies content.
    dstmeta.sensorStr = pyds.get_string(srcmeta.sensorStr)

    if srcmeta.objSignature.size > 0:
        dstmeta.objSignature.signature = pyds.memdup(
            srcmeta.objSignature.signature, srcmeta.objSignature.size)
        dstmeta.objSignature.size = srcmeta.objSignature.size

    if srcmeta.extMsgSize > 0:
        if srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE:
            srcobj = pyds.NvDsVehicleObject.cast(srcmeta.extMsg)
            obj = pyds.alloc_nvds_vehicle_object()
            obj.type = pyds.get_string(srcobj.type)
            obj.make = pyds.get_string(srcobj.make)
            obj.model = pyds.get_string(srcobj.model)
            obj.color = pyds.get_string(srcobj.color)
            obj.license = pyds.get_string(srcobj.license)
            obj.region = pyds.get_string(srcobj.region)
            dstmeta.extMsg = obj
            dstmeta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject)
        if srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON:
            srcobj = pyds.NvDsPersonObject.cast(srcmeta.extMsg)
            obj = pyds.alloc_nvds_person_object()
            obj.age = srcobj.age
            obj.gender = pyds.get_string(srcobj.gender)
            obj.cap = pyds.get_string(srcobj.cap)
            obj.hair = pyds.get_string(srcobj.hair)
            obj.apparel = pyds.get_string(srcobj.apparel)
            dstmeta.extMsg = obj
            dstmeta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject)

    return dstmeta


# Callback function for freeing an NvDsEventMsgMeta instance
def meta_free_func(data, user_data):
    user_meta = pyds.NvDsUserMeta.cast(data)
    srcmeta = pyds.NvDsEventMsgMeta.cast(user_meta.user_meta_data)

    # pyds.free_buffer takes C address of a buffer and frees the memory
    # It's a NOP if the address is NULL
    pyds.free_buffer(srcmeta.ts)
    pyds.free_buffer(srcmeta.sensorStr)

    if srcmeta.objSignature.size > 0:
        pyds.free_buffer(srcmeta.objSignature.signature)
        srcmeta.objSignature.size = 0

    if srcmeta.extMsgSize > 0:
        if srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE:
            obj = pyds.NvDsVehicleObject.cast(srcmeta.extMsg)
            pyds.free_buffer(obj.type)
            pyds.free_buffer(obj.color)
            pyds.free_buffer(obj.make)
            pyds.free_buffer(obj.model)
            pyds.free_buffer(obj.license)
            pyds.free_buffer(obj.region)
        if srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON:
            obj = pyds.NvDsPersonObject.cast(srcmeta.extMsg)
            pyds.free_buffer(obj.gender)
            pyds.free_buffer(obj.cap)
            pyds.free_buffer(obj.hair)
            pyds.free_buffer(obj.apparel)
        pyds.free_gbuffer(srcmeta.extMsg)
        srcmeta.extMsgSize = 0


def generate_DoorWarningSign_meta(data):
    obj = pyds.NvDsVehicleObject.cast(data)
    obj.type = "DoorWarningSign"
    obj.color = "y"
    obj.make = "B"
    obj.model = "M"
    obj.license = "l"
    obj.region = "CN"
    return obj


def generate_TwoWheeler_meta(data):
    obj = pyds.NvDsVehicleObject.cast(data)
    obj.type = "TwoWheeler"
    obj.color = "b"
    obj.make = "B"
    obj.model = "M"
    obj.license = "X"
    obj.region = "CN"
    return obj


def generate_person_meta(data):
    obj = pyds.NvDsPersonObject.cast(data)
    obj.age = 18
    obj.cap = "n"
    obj.hair = "b"
    obj.gender = "m"
    obj.apparel = "f"
    return obj


def generate_event_msg_meta(data, class_id):
    meta = pyds.NvDsEventMsgMeta.cast(data)
    meta.sensorId = 0
    meta.placeId = 0
    meta.moduleId = 0
    meta.sensorStr = sensor_id_str
    meta.ts = pyds.alloc_buffer(MAX_TIME_STAMP_LEN + 1)
    pyds.generate_ts_rfc3339(meta.ts, MAX_TIME_STAMP_LEN)

    # This demonstrates how to attach custom objects.
    # Any custom object as per requirement can be generated and attached
    # like NvDsVehicleObject / NvDsPersonObject. Then that object should
    # be handled in payload generator library (nvmsgconv.cpp) accordingly.
    if class_id == PGIE_CLASS_ID_DoorWarningSign:
        meta.type = pyds.NvDsEventType.NVDS_EVENT_ENTRY
        meta.objType = pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE
        meta.objClassId = PGIE_CLASS_ID_DoorWarningSign
        obj = pyds.alloc_nvds_vehicle_object()
        obj = generate_DoorWarningSign_meta(obj)
        meta.extMsg = obj
        meta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject)
    if class_id == PGIE_CLASS_ID_TwoWheeler:
        meta.type = pyds.NvDsEventType.NVDS_EVENT_ENTRY
        meta.objType = pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE
        meta.objClassId = PGIE_CLASS_ID_TwoWheeler
        obj = pyds.alloc_nvds_vehicle_object()
        obj = generate_TwoWheeler_meta(obj)
        meta.extMsg = obj
        meta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject)
    if class_id == PGIE_CLASS_ID_People:
        meta.type = pyds.NvDsEventType.NVDS_EVENT_ENTRY
        meta.objType = pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON
        meta.objClassId = PGIE_CLASS_ID_People
        obj = pyds.alloc_nvds_person_object()
        obj = generate_person_meta(obj)
        meta.extMsg = obj
        meta.extMsgSize = sys.getsizeof(pyds.NvDsPersonObject)

    return meta


# osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
# and update params for drawing rectangle, object information etc.
# IMPORTANT NOTE:
# a) probe() callbacks are synchronous and thus holds the buffer
#    (info.get_buffer()) from traversing the pipeline until user return.
# b) loops inside probe() callback could be costly in python.
#    So users shall optimize according to their use-case.
def osd_sink_pad_buffer_probe(pad, info, u_data):
    frame_number = 0
    # Intiallizing object counter with 0.
    obj_counter = {
        PGIE_CLASS_ID_DoorWarningSign: 0,
        PGIE_CLASS_ID_TwoWheeler: 0,
        PGIE_CLASS_ID_People: 0,
        # PGIE_CLASS_ID_ROADSIGN: 0
    }
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    if not batch_meta:
        return Gst.PadProbeReturn.OK
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            continue
        is_first_object = True

        # Short example of attribute access for frame_meta:
        # print("Frame Number is ", frame_meta.frame_num)
        # print("Source id is ", frame_meta.source_id)
        # print("Batch id is ", frame_meta.batch_id)
        # print("Source Frame Width ", frame_meta.source_frame_width)
        # print("Source Frame Height ", frame_meta.source_frame_height)
        # print("Num object meta ", frame_meta.num_obj_meta)

        frame_number = frame_meta.frame_num
        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                continue

            # Update the object text display
            txt_params = obj_meta.text_params

            # Set display_text. Any existing display_text string will be
            # freed by the bindings module.
            txt_params.display_text = pgie_classes_str[obj_meta.class_id]

            obj_counter[obj_meta.class_id] += 1

            # Font , font-color and font-size
            txt_params.font_params.font_name = "Serif"
            txt_params.font_params.font_size = 10
            # set(red, green, blue, alpha); set to White
            txt_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

            # Text background color
            txt_params.set_bg_clr = 1
            # set(red, green, blue, alpha); set to Black
            txt_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0)

            # Ideally NVDS_EVENT_MSG_META should be attached to buffer by the
            # component implementing detection / recognition logic.
            # Here it demonstrates how to use / attach that meta data.
            if is_first_object and (frame_number % 48) == 0:
                # Frequency of messages to be send will be based on use case.
                # Here message is being sent for first object every 30 frames.

                # Allocating an NvDsEventMsgMeta instance and getting
                # reference to it. The underlying memory is not manged by
                # Python so that downstream plugins can access it. Otherwise
                # the garbage collector will free it when this probe exits.
                msg_meta = pyds.alloc_nvds_event_msg_meta()
                msg_meta.bbox.top = obj_meta.rect_params.top
                msg_meta.bbox.left = obj_meta.rect_params.left
                msg_meta.bbox.width = obj_meta.rect_params.width
                msg_meta.bbox.height = obj_meta.rect_params.height
                msg_meta.frameId = frame_number
                msg_meta.trackingId = long_to_uint64(obj_meta.object_id)
                msg_meta.confidence = obj_meta.confidence
                msg_meta = generate_event_msg_meta(msg_meta, obj_meta.class_id)
                user_event_meta = pyds.nvds_acquire_user_meta_from_pool(
                    batch_meta)
                if user_event_meta:
                    print("A user_event_meta is made for ", pgie_classes_str[obj_meta.class_id], " will uploading...")
                    user_event_meta.user_meta_data = msg_meta
                    user_event_meta.base_meta.meta_type = pyds.NvDsMetaType.NVDS_EVENT_MSG_META
                    # Setting callbacks in the event msg meta. The bindings
                    # layer will wrap these callables in C functions.
                    # Currently only one set of callbacks is supported.
                    pyds.user_copyfunc(user_event_meta, meta_copy_func)
                    pyds.user_releasefunc(user_event_meta, meta_free_func)
                    pyds.nvds_add_user_meta_to_frame(frame_meta,
                                                     user_event_meta)
                else:
                    print("Error in attaching event meta to buffer\n")

                # disable it as we want all objects uploading
                # is_first_object = False
            try:
                l_obj = l_obj.next
            except StopIteration:
                break
        try:
            l_frame = l_frame.next
        except StopIteration:
            break
        # Get frame rate through this probe
        currentFps = fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()

        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        # Now set the offsets where the string should appear
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12

        # Font , font-color and font-size
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 10
        # set(red, green, blue, alpha); set to White
        py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

        # Text background color
        py_nvosd_text_params.set_bg_clr = 1
        # set(red, green, blue, alpha); set to Black
        py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0)
        py_nvosd_text_params.display_text = "Fps: {0}".format(currentFps)
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        print("Fps: {0}".format(currentFps), "FraNo.:", frame_number, "DWS:",
              obj_counter[PGIE_CLASS_ID_DoorWarningSign],
              "EB:", obj_counter[PGIE_CLASS_ID_TwoWheeler], "Pep", obj_counter[PGIE_CLASS_ID_People])

    return Gst.PadProbeReturn.OK


def cb_newpad(decodebin, decoder_src_pad, data):
    print("In cb_newpad\n")
    caps = decoder_src_pad.get_current_caps()
    gststruct = caps.get_structure(0)
    gstname = gststruct.get_name()
    source_bin = data
    features = caps.get_features(0)

    # Need to check if the pad created by the decodebin is for video and not
    # audio.
    print("gstname=", gstname)
    if gstname.find("video") != -1:
        # Link the decodebin pad only if decodebin has picked nvidia
        # decoder plugin nvdec_*. We do this by checking if the pad caps contain
        # NVMM memory features.
        print("features=", features)
        if features.contains("memory:NVMM"):
            # Get the source bin ghost pad
            bin_ghost_pad = source_bin.get_static_pad("src")
            if not bin_ghost_pad.set_target(decoder_src_pad):
                sys.stderr.write(
                    "Failed to link decoder src pad to source bin ghost pad\n"
                )
        else:
            sys.stderr.write(
                " Error: Decodebin did not pick nvidia decoder plugin.\n")


def decodebin_child_added(child_proxy, Object, name, user_data):
    print("Decodebin child added:", name, "\n")
    if name.find("decodebin") != -1:
        Object.connect("child-added", decodebin_child_added, user_data)


def create_source_bin(index, uri):
    # print("     Creating source bin")

    # Create a source GstBin to abstract this bin's content from the rest of the
    # pipeline
    bin_name = "source-bin-%02d" % index
    print("     creating source bin with name:", bin_name)
    # print(bin_name)
    nbin = Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    # Source element for reading from the uri.
    # We will use decodebin and let it figure out the container format of the
    # stream and the codec and plug the appropriate demux and decode plugins.
    uri_decode_bin = Gst.ElementFactory.make("uridecodebin", "uri-decode-bin")
    if not uri_decode_bin:
        sys.stderr.write(" Unable to create uri decode bin \n")
    # We set the input uri to the source element
    uri_decode_bin.set_property("uri", uri)
    # Connect to the "pad-added" signal of the decodebin which generates a
    # callback once a new pad for raw data has beed created by the decodebin
    uri_decode_bin.connect("pad-added", cb_newpad, nbin)
    uri_decode_bin.connect("child-added", decodebin_child_added, nbin)

    # We need to create a ghost pad for the source bin which will act as a proxy
    # for the video decoder src pad. The ghost pad will not have a target right
    # now. Once the decode bin creates the video decoder and generates the
    # cb_newpad callback, we will set the ghost pad target to the video decoder
    # src pad.
    Gst.Bin.add(nbin, uri_decode_bin)
    bin_pad = nbin.add_pad(
        Gst.GhostPad.new_no_target(
            "src", Gst.PadDirection.SRC))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin


def main(args):
    GObject.threads_init()
    Gst.init(None)

    # registering callbacks
    pyds.register_user_copyfunc(meta_copy_func)
    pyds.register_user_releasefunc(meta_free_func)

    print("Creating Pipeline \n ")

    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")

    print("Creating streamux \n ")
    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")
    pipeline.add(streammux)

    number_sources = 0
    for i in range(len(args)):
        uri_name = args[i]
        if uri_name.find("rtsp://") == 0:
            is_live = True
        elif uri_name.find("file://") == 0:
            pass
        else:
            continue

        print("Creating source_bin", number_sources)
        source_bin = create_source_bin(number_sources, uri_name)
        if not source_bin:
            sys.stderr.write("Unable to create source bin \n")
        pipeline.add(source_bin)
        padname = "sink_%u" % number_sources
        sinkpad = streammux.get_request_pad(padname)
        if not sinkpad:
            sys.stderr.write("Unable to create sink pad bin \n")
        srcpad = source_bin.get_static_pad("src")
        if not srcpad:
            sys.stderr.write("Unable to create src pad bin \n")
        srcpad.link(sinkpad)

        fps_streams["stream{0}".format(number_sources)] = GETFPS(number_sources)
        number_sources += 1
        # Shawn, we only support single one source for now, otherwise, the tiler element is required.
        break

    # print("Creating H264Parser \n")
    # h264parser = Gst.ElementFactory.make("h264parse", "h264-parser")
    # if not h264parser:
    #     sys.stderr.write(" Unable to create h264 parser \n")
    #
    # print("Creating Decoder \n")
    # decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
    # if not decoder:
    #     sys.stderr.write(" Unable to create Nvv4l2 Decoder \n")
    #
    # streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    # if not streammux:
    #     sys.stderr.write(" Unable to create NvStreamMux \n")

    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")

    # print("Creating tiler \n ")
    # tiler = Gst.ElementFactory.make("nvmultistreamtiler", "nvtiler")
    # if not tiler:
    #     sys.stderr.write(" Unable to create tiler \n")

    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")

    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")

    msgconv = Gst.ElementFactory.make("nvmsgconv", "nvmsg-converter")
    if not msgconv:
        sys.stderr.write(" Unable to create msgconv \n")

    msgbroker = Gst.ElementFactory.make("nvmsgbroker", "nvmsg-broker")
    if not msgbroker:
        sys.stderr.write(" Unable to create msgbroker \n")

    tee = Gst.ElementFactory.make("tee", "nvsink-tee")
    if not tee:
        sys.stderr.write(" Unable to create tee \n")

    queue1 = Gst.ElementFactory.make("queue", "nvtee-que1")
    if not queue1:
        sys.stderr.write(" Unable to create queue1 \n")

    queue2 = Gst.ElementFactory.make("queue", "nvtee-que2")
    if not queue2:
        sys.stderr.write(" Unable to create queue2 \n")

    if no_display:
        if no_output_rtsp:
            print("Creating FakeSink \n")
            sink = Gst.ElementFactory.make("fakesink", "fakesink")
            if not sink:
                sys.stderr.write(" Unable to create fakesink \n")
        else:
            # shawn, we create a RTSP out here
            nvvidconv_postosd = Gst.ElementFactory.make("nvvideoconvert", "convertor_postosd")
            if not nvvidconv_postosd:
                sys.stderr.write(" Unable to create nvvidconv_postosd \n")
            # Create a caps filter
            caps = Gst.ElementFactory.make("capsfilter", "filter")
            caps.set_property("caps", Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420"))

            codec = "H264"
            # default bitrate is 4000000
            bitrate = 3000000
            # Make the encoder
            if codec == "H264":
                encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
                print("Creating H264 Encoder")
            elif codec == "H265":
                encoder = Gst.ElementFactory.make("nvv4l2h265enc", "encoder")
                print("Creating H265 Encoder")
            if not encoder:
                sys.stderr.write(" Unable to create encoder")
            encoder.set_property('bitrate', bitrate)
            if is_aarch64():
                encoder.set_property('preset-level', 1)
                encoder.set_property('insert-sps-pps', 1)
                encoder.set_property('bufapi-version', 1)

            # Make the payload-encode video into RTP packets
            if codec == "H264":
                rtppay = Gst.ElementFactory.make("rtph264pay", "rtppay")
                print("Creating H264 rtppay")
            elif codec == "H265":
                rtppay = Gst.ElementFactory.make("rtph265pay", "rtppay")
                print("Creating H265 rtppay")
            if not rtppay:
                sys.stderr.write(" Unable to create rtppay")

            # Make the UDP sink
            updsink_port_num = 5400
            sink = Gst.ElementFactory.make("udpsink", "udpsink")
            if not sink:
                sys.stderr.write(" Unable to create udpsink")

            sink.set_property('host', '224.224.255.255')
            sink.set_property('port', updsink_port_num)
            sink.set_property('async', False)
            sink.set_property('sync', 1)

            pipeline.add(nvvidconv_postosd)
            pipeline.add(caps)
            pipeline.add(encoder)
            pipeline.add(rtppay)
    else:
        if is_aarch64():
            transform = Gst.ElementFactory.make("nvegltransform",
                                                "nvegl-transform")

        print("Creating EGLSink \n")
        sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
        if not sink:
            sys.stderr.write(" Unable to create egl sink \n")

    print("Playing input src URI(s)-> %s " % input_src_uri)
    # source.set_property('location', input_file)
    pgie_config_file_parser = configparser.ConfigParser()
    if pgie_config_file_parser.read(pgie_config_file) and pgie_config_file_parser['property']['input-dims']:
        pgie_input_dims_str = pgie_config_file_parser['property']['input-dims']
        streammux.set_property('width', int(pgie_input_dims_str.split(";")[2]))
        streammux.set_property('height', int(pgie_input_dims_str.split(";")[1]))
    else:
        streammux.set_property('width', 1280)
        streammux.set_property('height', 720)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)

    pgie.set_property('config-file-path', pgie_config_file)
    pgie_batch_size = pgie.get_property("batch-size")
    if pgie_batch_size != number_sources:
        print(
            "WARNING: Overriding infer-config batch-size",
            pgie_batch_size,
            " with number of sources",
            number_sources,
            " \n",
        )
        pgie.set_property("batch-size", number_sources)

    print("Uploading data with Kafka plugin-> %s " % proto_lib)
    print("     Uploading data with Kafka conn_str-> %s " % conn_str)
    msgconv.set_property('config', MSCONV_CONFIG_FILE)
    msgconv.set_property('payload-type', schema_type)
    msgbroker.set_property('proto-lib', proto_lib)
    msgbroker.set_property('conn-str', conn_str)
    if cfg_file is not None:
        msgbroker.set_property('config', cfg_file)
    if topic is not None:
        msgbroker.set_property('topic', topic)
    msgbroker.set_property('sync', False)

    print("Adding elements to Pipeline \n")
    # pipeline.add(source)
    # pipeline.add(h264parser)
    # pipeline.add(decoder)
    # pipeline.add(streammux)

    # tiler_rows = int(math.sqrt(number_sources))
    # tiler_columns = int(math.ceil((1.0 * number_sources) / tiler_rows))
    # tiler.set_property("rows", tiler_rows)
    # tiler.set_property("columns", tiler_columns)
    # tiler.set_property("width", TILED_OUTPUT_WIDTH)
    # tiler.set_property("height", TILED_OUTPUT_HEIGHT)

    sink.set_property("qos", 0)

    pipeline.add(pgie)
    # pipeline.add(tiler)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(tee)
    pipeline.add(queue1)
    pipeline.add(queue2)
    pipeline.add(msgconv)
    pipeline.add(msgbroker)
    pipeline.add(sink)
    if is_aarch64() and not no_display:
        pipeline.add(transform)

    print("Linking elements in the Pipeline \n")
    # source.link(h264parser)
    # h264parser.link(decoder)

    # sinkpad = streammux.get_request_pad("sink_0")
    # if not sinkpad:
    #     sys.stderr.write(" Unable to get the sink pad of streammux \n")
    # srcpad = decoder.get_static_pad("src")
    # if not srcpad:
    #     sys.stderr.write(" Unable to get source pad of decoder \n")
    # srcpad.link(sinkpad)

    streammux.link(pgie)
    pgie.link(nvvidconv)
    # nvvidconv.link(tiler)
    # tiler.link(nvosd)
    nvvidconv.link(nvosd)
    nvosd.link(tee)
    queue1.link(msgconv)
    msgconv.link(msgbroker)
    if is_aarch64() and not no_display:
        queue2.link(transform)
        transform.link(sink)
    else:
        if no_output_rtsp:
            queue2.link(sink)
        else:
            queue2.link(nvvidconv_postosd)
            # nvosd.link(nvvidconv_postosd)
            nvvidconv_postosd.link(caps)
            caps.link(encoder)
            encoder.link(rtppay)
            rtppay.link(sink)
    sink_pad = queue1.get_static_pad("sink")
    tee_msg_pad = tee.get_request_pad('src_%u')
    tee_render_pad = tee.get_request_pad("src_%u")
    if not tee_msg_pad or not tee_render_pad:
        sys.stderr.write("Unable to get request pads\n")
    tee_msg_pad.link(sink_pad)
    sink_pad = queue2.get_static_pad("sink")
    tee_render_pad.link(sink_pad)

    # create an event loop and feed gstreamer bus messages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)
    if no_display and not no_output_rtsp:
        # Start streaming
        rtsp_port_num = 8554

        server = GstRtspServer.RTSPServer.new()
        server.props.service = "%d" % rtsp_port_num
        server.attach(None)

        factory = GstRtspServer.RTSPMediaFactory.new()
        factory.set_launch(
            "( udpsrc name=pay0 port=%d buffer-size=524288 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)%s, payload=96 \" )" % (
                updsink_port_num, codec))
        factory.set_shared(True)
        server.get_mount_points().add_factory("/eow", factory)

        print("\n *** DeepStream: Launched RTSP Streaming output at rtsp://localhost:%d/eow ***\n\n" % rtsp_port_num)

    osdsinkpad = nvosd.get_static_pad("sink")
    if not osdsinkpad:
        sys.stderr.write(" Unable to get sink pad of nvosd \n")

    osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)

    # tiler_src_pad = pgie.get_static_pad("src")
    # if not tiler_src_pad:
    #     sys.stderr.write(" Unable to get src pad \n")
    # else:
    #     tiler_src_pad.add_probe(Gst.PadProbeType.BUFFER, tiler_src_pad_buffer_probe, 0)

    print("Starting pipeline \n")

    # start play back and listed to events
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup
    pyds.unset_callback_funcs()
    pipeline.set_state(Gst.State.NULL)


# Parse and validate input arguments
def parse_args():
    parser = OptionParser()
    parser.add_option("-c", "--cfg-file", dest="cfg_file",
                      help="Set the adaptor config file. Optional if "
                           "connection string has relevant  details.",
                      metavar="FILE")
    parser.add_option("-i", "--input-src-uri", dest="input_src_uri",
                      help="Set the input source uri(rtsp:// or file://))",
                      # default="file:///opt/nvidia/deepstream/deepstream/samples/streams/sample_720p.mp4",
                      metavar="FILE")
    parser.add_option("-p", "--proto-lib", dest="proto_lib",
                      help="Absolute path of adaptor library",
                      default="/opt/nvidia/deepstream/deepstream/lib/libnvds_kafka_proto.so",
                      metavar="PATH")
    parser.add_option("", "--conn-str", dest="conn_str",
                      help="Connection string of backend server. Optional if "
                           "it is part of config file.",
                      default="dev-iot.ipos.biz;9092",
                      metavar="STR")
    parser.add_option("-s", "--schema-type", dest="schema_type", default="1",
                      help="Type of message schema (0=Full, 1=minimal), "
                           "default=0", metavar="<0|1>")
    parser.add_option("-t", "--topic", dest="topic",
                      help="Name of message topic. Optional if it is part of "
                           "connection string or config file.",
                      default="test",
                      metavar="TOPIC")
    parser.add_option("", "--no-display", action="store_true",
                      dest="no_display", default=False,
                      help="Disable local display window")
    parser.add_option("", "--no-output-rtsp", action="store_true",
                      dest="no_output_rtsp", default=False,
                      help="Disable output result to a rtsp stream")
    parser.add_option("", "--pgie-config-file", dest="pgie_config_file",
                      help="pgie file path",
                      default="config_infer_primary_elenet.txt",
                      metavar="STR")

    (options, args) = parser.parse_args()

    global cfg_file
    global input_src_uri
    global proto_lib
    global conn_str
    global topic
    global sensor_id_str
    global schema_type
    global no_display
    global no_output_rtsp
    global pgie_config_file
    cfg_file = options.cfg_file
    input_src_uri = options.input_src_uri
    proto_lib = options.proto_lib
    conn_str = options.conn_str
    topic = options.topic
    no_display = options.no_display
    no_output_rtsp = options.no_output_rtsp
    pgie_config_file = options.pgie_config_file

    if not (proto_lib and input_src_uri):
        print("Usage: python3 deepstream_test_51.py -i <rtsp:// or file://> -p "
              "<Proto adaptor library> --conn-str=<Connection string>")
        return 1

    if not os.path.exists('/opt/nvidia/deepstream/deepstream-6.0/samples/configs/deepstream-app/config_elenet.txt'):
        print("Could not find config_elenet.txt under "
              "/opt/nvidia/deepstream/deepstream-6.0/samples/configs/deepstream-app/")
        return 1
    config = configparser.ConfigParser()
    config.read('/opt/nvidia/deepstream/deepstream-6.0/samples/configs/deepstream-app/config_elenet.txt')
    sensor_id_str = config['custom-uploader']['whoami']
    schema_type = 0 if options.schema_type == "0" else 1


if __name__ == '__main__':
    ret = parse_args()
    # If argument parsing fails, returns failure (non-zero)
    if ret == 1:
        sys.exit(1)
    sys.exit(main(sys.argv))
