import network
import os
import time
import _thread
import gc
import sys
import uctypes
import multimedia as mm

from time import sleep
from machine import UART, FPIOA
from media.vencoder import *
from media.sensor import *
from media.media import *
from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11


WIFI_SSID = "canMV"
WIFI_PASSWORD = "111111111"

kmodel_path = "/sdcard/yolo1320.kmodel"
labels = {0: "0"}
model_input_size = [320, 320]

display = "lcd2_4"
rgb888p_size = [640, 360]
rtsp_size = [512, 288]

confidence_threshold = 0.28
nms_threshold = 0.45
offset_range = 12.5
uart2 = None
wifi_ip = "0.0.0.0"
kalman_x = 0.0
kalman_vx = 0.0
kalman_p00 = 1.0
kalman_p01 = 0.0
kalman_p10 = 0.0
kalman_p11 = 1.0
kalman_inited = False
predict_lost_count = 0
kalman_lead_frames = 1.5
kalman_q_pos = 0.08
kalman_q_vel = 0.35
kalman_r = 18.0
max_predict_shift = 80

try:
    RTSP_CHN_ID = CAM_CHN_ID_1
except NameError:
    RTSP_CHN_ID = 1


def Connect_WIFI(ID, PASSWORD, max_retries=3, timeout_s=15):
    global wifi_ip
    sta = network.WLAN(0)

    for attempt in range(1, max_retries + 1):
        print("[WIFI] Connect attempt {}/{}".format(attempt, max_retries))

        try:
            sta.disconnect()
        except BaseException:
            pass

        sta.active(False)
        time.sleep_ms(500)
        sta.active(True)
        time.sleep_ms(500)

        sta.connect(ID, PASSWORD)
        for _ in range(timeout_s):
            if sta.isconnected() and sta.ifconfig()[0] != "0.0.0.0":
                wifi_ip = sta.ifconfig()[0]
                print("[WIFI] IP:", wifi_ip)
                return True
            time.sleep(1)

        print("[WIFI] Attempt {} timed out".format(attempt))

    wifi_ip = "0.0.0.0"
    return False


def init_uart2():
    try:
        fpioa = FPIOA()
        fpioa.set_function(11, FPIOA.UART2_TXD)
        fpioa.set_function(12, FPIOA.UART2_RXD)
        try:
            uart = UART(
                UART.UART2,
                baudrate=115200,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE,
            )
        except BaseException:
            uart = UART(UART.UART2, baudrate=115200)
        print("[UART2] TX=11 RX=12 baud=115200")
        return uart
    except BaseException as e:
        print("[UART2] init failed:", e)
        return None


def uart2_send(text):
    if uart2:
        try:
            uart2.write(text)
        except BaseException as e:
            print("[UART2] write failed:", e)


def parse_detect_box(box):
    if len(box) < 6:
        return None

    if box[1] <= 1.0 and box[2] > 1:
        cls_id = int(box[0])
        score = box[1]
        x1, y1, x2, y2 = box[2], box[3], box[4], box[5]
    elif box[4] <= 1.0:
        x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
        score = box[4]
        cls_id = int(box[5])
    else:
        cls_id = int(box[0])
        score = box[1]
        x1, y1, x2, y2 = box[2], box[3], box[4], box[5]

    return {
        "cls_id": cls_id,
        "score": score,
        "x1": int(x1),
        "y1": int(y1),
        "x2": int(x2),
        "y2": int(y2),
    }


def select_best_ball(res):
    if len(res) >= 3 and isinstance(res[0], list) and isinstance(res[1], list) and isinstance(res[2], list):
        boxes, cls_ids, scores = res[0], res[1], res[2]
        best = None
        for i in range(len(boxes)):
            if i >= len(cls_ids) or i >= len(scores):
                continue
            if int(cls_ids[i]) != 0:
                continue
            box = boxes[i]
            if len(box) < 4:
                continue
            x, y, w, h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
            det = {
                "cls_id": int(cls_ids[i]),
                "score": scores[i],
                "x1": x,
                "y1": y,
                "x2": x + w,
                "y2": y + h,
            }
            if best is None or det["score"] > best["score"]:
                best = det
        return best

    best = None
    for box in res:
        det = parse_detect_box(box)
        if det is None:
            continue
        if det["cls_id"] != 0:
            continue
        if best is None or det["score"] > best["score"]:
            best = det
    return best


def calc_horizontal_offset(det, display_size):
    ball_center_x = (det["x1"] + det["x2"]) / 2
    screen_center_x = display_size[0] / 2
    dx_pixel = screen_center_x - ball_center_x
    return dx_pixel / screen_center_x * offset_range


def reset_ball_predictor():
    global kalman_x, kalman_vx, kalman_p00, kalman_p01, kalman_p10, kalman_p11
    global kalman_inited, predict_lost_count
    kalman_x = 0.0
    kalman_vx = 0.0
    kalman_p00 = 1.0
    kalman_p01 = 0.0
    kalman_p10 = 0.0
    kalman_p11 = 1.0
    kalman_inited = False
    predict_lost_count = 0


def predict_ball_horizontal(det):
    global kalman_x, kalman_vx, kalman_p00, kalman_p01, kalman_p10, kalman_p11
    global kalman_inited, predict_lost_count

    measured_x = (det["x1"] + det["x2"]) / 2
    box_w = det["x2"] - det["x1"]
    visible_w = rgb888p_size[0]

    if not kalman_inited:
        kalman_x = measured_x
        kalman_vx = 0.0
        kalman_p00 = 20.0
        kalman_p01 = 0.0
        kalman_p10 = 0.0
        kalman_p11 = 10.0
        kalman_inited = True
    else:
        pred_x = kalman_x + kalman_vx
        pred_vx = kalman_vx

        pred_p00 = kalman_p00 + kalman_p01 + kalman_p10 + kalman_p11 + kalman_q_pos
        pred_p01 = kalman_p01 + kalman_p11
        pred_p10 = kalman_p10 + kalman_p11
        pred_p11 = kalman_p11 + kalman_q_vel

        residual = measured_x - pred_x
        residual_cov = pred_p00 + kalman_r
        k0 = pred_p00 / residual_cov
        k1 = pred_p10 / residual_cov

        kalman_x = pred_x + k0 * residual
        kalman_vx = pred_vx + k1 * residual
        kalman_p00 = (1.0 - k0) * pred_p00
        kalman_p01 = (1.0 - k0) * pred_p01
        kalman_p10 = pred_p10 - k1 * pred_p00
        kalman_p11 = pred_p11 - k1 * pred_p01

    predict_lost_count = 0
    shift = kalman_vx * kalman_lead_frames
    if shift > max_predict_shift:
        shift = max_predict_shift
    elif shift < -max_predict_shift:
        shift = -max_predict_shift

    predicted_center_x = kalman_x + shift
    half_w = box_w / 2
    if predicted_center_x < half_w:
        predicted_center_x = half_w
    elif predicted_center_x > visible_w - half_w:
        predicted_center_x = visible_w - half_w

    predicted = det.copy()
    predicted["x1"] = int(predicted_center_x - half_w)
    predicted["x2"] = int(predicted_center_x + half_w)
    return predicted


def draw_center_cross(osd_img, display_size):
    cx = display_size[0] // 2
    cy = display_size[1] // 2
    try:
        osd_img.draw_cross(cx, cy, color=(255, 0, 0, 255), size=14, thickness=2)
    except BaseException:
        try:
            osd_img.draw_line(cx - 10, cy, cx + 10, cy, color=(255, 0, 0, 255), thickness=2)
            osd_img.draw_line(cx, cy - 10, cx, cy + 10, color=(255, 0, 0, 255), thickness=2)
        except TypeError:
            osd_img.draw_line(cx - 10, cy, cx + 10, cy, color=(255, 0, 0), thickness=2)
            osd_img.draw_line(cx, cy - 10, cx, cy + 10, color=(255, 0, 0), thickness=2)


def draw_offset_text(osd_img, x, y, text):
    try:
        osd_img.draw_string(x + 1, y + 1, text, color=(0, 0, 0, 255), scale=3)
        osd_img.draw_string(x, y, text, color=(255, 255, 255, 255), scale=3)
    except TypeError:
        osd_img.draw_string(x + 1, y + 1, text, color=(0, 0, 0), scale=3)
        osd_img.draw_string(x, y, text, color=(255, 255, 255), scale=3)


def draw_ball_offset(osd_img, det, offset, display_size):
    x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
    w = x2 - x1
    h = y2 - y1
    text = "X=%.2f" % offset

    try:
        osd_img.draw_rectangle(x1, y1, w, h, color=(255, 255, 0, 255), thickness=3)
    except TypeError:
        osd_img.draw_rectangle(x1, y1, w, h, color=(255, 255, 0), thickness=3)

    try:
        osd_img.draw_cross((x1 + x2) // 2, (y1 + y2) // 2, color=(255, 0, 0, 255), size=12, thickness=2)
    except BaseException:
        pass

    visible_w = rgb888p_size[0]
    visible_h = rgb888p_size[1]
    text_x = max(0, min(x1, visible_w - 120))
    text_y = y2 + 4
    if text_y > visible_h - 32:
        text_y = max(0, y1 - 28)

    draw_offset_text(osd_img, text_x, text_y, text)


def draw_status_text(osd_img):
    try:
        osd_img.draw_string(8, 8, "IP " + wifi_ip, color=(255, 255, 255, 255), scale=2)
    except TypeError:
        osd_img.draw_string(8, 8, "IP " + wifi_ip, color=(255, 255, 255), scale=2)


def handle_ball_result(res, osd_img, display_size):
    global predict_lost_count
    draw_status_text(osd_img)
    draw_center_cross(osd_img, display_size)
    det = select_best_ball(res)
    if det is None:
        predict_lost_count += 1
        if predict_lost_count >= 5:
            reset_ball_predictor()
        uart2_send("none\n")
        return None

    pred_det = predict_ball_horizontal(det)
    offset = calc_horizontal_offset(pred_det, display_size)
    draw_ball_offset(osd_img, pred_det, offset, display_size)
    uart2_send("%.2f\n" % offset)
    return offset


def configure_rtsp_channel(sensor, width, height):
    width = ALIGN_UP(width, 16)
    try:
        sensor.set_framesize(width=width, height=height, alignment=12, chn=RTSP_CHN_ID)
        sensor.set_pixformat(Sensor.YUV420SP, chn=RTSP_CHN_ID)
        print("[RTSP] Sensor channel configured:", width, height)
        return True
    except BaseException as e:
        print("[RTSP] Sensor channel config skipped:", e)
        return False


def snapshot_rtsp_frame(sensor):
    try:
        return sensor.snapshot()
    except TypeError:
        return sensor.snapshot()
    except BaseException as e:
        print("[RTSP] snapshot channel failed:", e)
        try:
            return sensor.snapshot()
        except BaseException as e2:
            print("[RTSP] snapshot default failed:", e2)
            return -1


class RtspServer:
    def __init__(
        self,
        sensor,
        session_name="video",
        port=8554,
        video_type=mm.multi_media_type.media_h264,
        enable_audio=False,
    ):
        self.session_name = session_name
        self.video_type = video_type
        self.enable_audio = enable_audio
        self.port = port
        self.rtspserver = mm.rtsp_server()
        self.venc_chn = VENC_CHN_ID_0
        self.start_stream = False
        self.runthread_over = False
        self.sensor = sensor
        self.encoder = Encoder()
        self.link = None
        self.width = ALIGN_UP(rtsp_size[0], 16)
        self.height = rtsp_size[1]
        self.outbufs_ready = False
        self.sent_frames = 0
        self.sent_packets = 0
        self.zero_packets = 0
        self.rtsp_timestamp = 0
        self.debug_logged = False

    def start(self):
        try:
            self._create_stream()
            self.rtspserver.rtspserver_init(self.port)
            self.rtspserver.rtspserver_createsession(
                self.session_name, self.video_type, self.enable_audio
            )
            self.rtspserver.rtspserver_start()
            self.encoder.Start(self.venc_chn)
        except BaseException as e:
            print("[RTSP] Start failed:", e)
            return False

        self.start_stream = True
        _thread.start_new_thread(self._do_rtsp_stream, ())
        return True

    def stop(self):
        if self.start_stream == False:
            return

        self.start_stream = False
        while not self.runthread_over:
            sleep(0.1)
        self.runthread_over = False

        try:
            self.encoder.Stop(self.venc_chn)
            self.encoder.Destroy(self.venc_chn)
        except BaseException as e:
            print("[RTSP] encoder stop failed:", e)

        try:
            self.rtspserver.rtspserver_stop()
            self.rtspserver.rtspserver_deinit()
        except BaseException as e:
            print("[RTSP] server stop failed:", e)

    def get_rtsp_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)

    def prepare_buffers(self):
        if self.outbufs_ready:
            return
        self.encoder.SetOutBufs(self.venc_chn, 8, self.width, self.height)
        self.outbufs_ready = True

    def _create_stream(self):
        self.prepare_buffers()
        chnAttr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            self.width,
            self.height,
            bit_rate=100,
            dst_frame_rate=5,
            src_frame_rate=5,
        )
        self.encoder.Create(self.venc_chn, chnAttr)

    def _fill_frame_info(self, img, frame_info):
        frame_info.v_frame.width = img.width()
        frame_info.v_frame.height = img.height()
        frame_info.v_frame.pixel_format = Sensor.YUV420SP
        frame_info.pool_id = img.poolid()
        frame_info.v_frame.phys_addr[0] = img.phyaddr()

        if img.width() == 800 and img.height() == 480:
            offset = img.width() * img.height() + 1024
        elif img.width() == 1920 and img.height() == 1080:
            offset = img.width() * img.height() + 3072
        elif img.width() == 640 and img.height() == 360:
            offset = img.width() * img.height() + 3072
        else:
            offset = img.width() * img.height()

        frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + offset

    def _do_rtsp_stream(self):
        try:
            streamData = StreamData()
            frame_info = k_video_frame_info()

            while self.start_stream:
                rtsp_img = snapshot_rtsp_frame(self.sensor)
                if rtsp_img == -1:
                    if self.sent_frames % 30 == 0:
                        print("[RTSP] No frame from sensor")
                    self.sent_frames += 1
                    time.sleep_ms(10)
                    continue

                if not self.debug_logged:
                    print("[RTSP] First frame:", rtsp_img.width(), rtsp_img.height(), rtsp_img.poolid())
                self._fill_frame_info(rtsp_img, frame_info)
                self.encoder.SendFrame(self.venc_chn, frame_info)
                self.encoder.GetStream(self.venc_chn, streamData)
                if not self.debug_logged:
                    print("[RTSP] First stream pack count:", streamData.pack_cnt)

                sent_this_frame = 0
                for pack_idx in range(0, streamData.pack_cnt):
                    packet_size = streamData.data_size[pack_idx]
                    if packet_size <= 0:
                        self.zero_packets += 1
                        if self.zero_packets <= 10 or self.zero_packets % 30 == 0:
                            print("[RTSP] Zero packet:", self.zero_packets)
                        continue

                    stream_data = bytes(
                        uctypes.bytearray_at(
                            streamData.data[pack_idx],
                            packet_size,
                        )
                    )
                    self.rtspserver.rtspserver_sendvideodata(
                        self.session_name,
                        stream_data,
                        packet_size,
                        self.rtsp_timestamp,
                    )
                    sent_this_frame += 1
                    self.sent_packets += 1
                    if not self.debug_logged:
                        print("[RTSP] First nonzero packet size:", packet_size)

                self.encoder.ReleaseStream(self.venc_chn, streamData)
                if sent_this_frame > 0:
                    self.debug_logged = True
                    self.rtsp_timestamp += 100
                self.sent_frames += 1
                if self.sent_frames % 30 == 0:
                    print("[RTSP] Encoded frames:", self.sent_frames, "sent packets:", self.sent_packets)
                gc.collect()
                time.sleep_us(10)
                os.exitpoint()
        except BaseException as e:
            print("[RTSP] Exception:", e)
        finally:
            self.runthread_over = True


def get_display_config():
    if display == "hdmi":
        return "hdmi", [1920, 1080]
    if display == "lcd3_5":
        return "st7701", [800, 480]
    return "st7701", [640, 480]


def create_pipeline():
    display_mode, display_size = get_display_config()
    sensor = Sensor(width=1280, height=960)
    rtspserver = RtspServer(sensor)
    rtspserver.prepare_buffers()

    pl = PipeLine(
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        display_mode=display_mode,
    )
    pl.create(sensor=sensor)
    return pl, sensor, rtspserver


def main():
    global uart2
    uart2 = init_uart2()

    print("[WIFI] Connecting to network ...")
    isConnected = Connect_WIFI(WIFI_SSID, WIFI_PASSWORD)
    if isConnected:
        print("[WIFI] Network connection successful")
    else:
        print("[WIFI] Network connection failed! Please check the configuration")
        time.sleep_ms(10)
        sys.exit()

    pl = None
    yolo = None
    rtspserver = None

    try:
        pl, sensor, rtspserver = create_pipeline()
        display_size = pl.get_display_size()

        yolo = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=kmodel_path,
            labels=labels,
            rgb888p_size=rgb888p_size,
            model_input_size=model_input_size,
            display_size=display_size,
            conf_thresh=confidence_threshold,
            nms_thresh=nms_threshold,
            max_boxes_num=50,
            debug_mode=0,
        )
        yolo.config_preprocess()

        clock = time.clock()
        frame_count = 0
        rtsp_started = False
        while True:
            clock.tick()
            img = pl.get_frame()
            res = yolo.run(img)
            try:
                pl.osd_img.clear()
            except BaseException:
                pass
            offset = handle_ball_result(res, pl.osd_img, display_size)
            print("[YOLO]", res, "offset:", offset)
            pl.show_image()
            frame_count += 1

            if rtspserver and not rtsp_started and frame_count >= 5:
                print("[RTSP] Starting ...")
                if rtspserver.start():
                    rtsp_url = rtspserver.get_rtsp_url()
                    print("[RTSP] Started successfully")
                    print("[RTSP] Open this URL in VLC:", rtsp_url)
                else:
                    print("[RTSP] Disabled after start failure")
                rtsp_started = True

            gc.collect()
            print("FPS:", clock.fps())
            time.sleep_ms(1)
    finally:
        if rtspserver:
            rtspserver.stop()
        if yolo:
            yolo.deinit()
        if pl:
            pl.destroy()


if __name__ == "__main__":
    main()
