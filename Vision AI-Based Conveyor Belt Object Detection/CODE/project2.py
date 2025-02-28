from flask import Flask, Response, render_template
import serial
import requests
import numpy as np
from io import BytesIO
import cv2
from requests.auth import HTTPBasicAuth
import time

app = Flask(__name__)

# Vision API 및 인증 정보
VISION_API_URL = "https://suite-endpoint-api-apne2.superb-ai.com/endpoints/4317c958-c251-4945-9b91-e11cafbbd1e7/inference"
TEAM = "kdt2025_1-16"
ACCESS_KEY = "10lLkiLFq04srPrueQp3t5c6AxkcCRbbacbS8NQO"

# 색상 매핑
colors = {
    "Raspberry PICO": (255, 0, 0),
    "Oscilator": (0, 255, 0),
    "ChipSet": (255, 255, 0),
    "USb": (0, 255, 255),
    "Hole": (0, 0, 255),
    "BootSel": (0, 0, 255),
    "Header": (255, 0, 255),
}

# 부품 카운트 추적 변수
required_parts = {
    "Hole": 4,
    "Raspberry PICO": 1,
    "Oscilator": 1,
    "ChipSet": 1,
    "USb": 1,
    "BootSel": 1,
    "Header": 1,
}

# 아두이노와 직렬 통신 설정
ser = serial.Serial("/dev/ttyACM0", 9600)

# 웹캠 초기화
camera = cv2.VideoCapture(0)

# 상태 메시지 변수
status_message = "Waiting for Arduino signal..."


def inference_request(img: np.array):
    """API로 이미지를 전송하고 응답을 반환"""
    _, img_encoded = cv2.imencode(".jpg", img)
    img_bytes = BytesIO(img_encoded.tobytes())
    try:
        response = requests.post(
            url=VISION_API_URL,
            auth=HTTPBasicAuth(TEAM, ACCESS_KEY),
            headers={"Content-Type": "image/jpeg"},
            data=img_bytes,
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")
        return None


def draw_bounding_boxes(frame, objects):
    """감지된 객체의 바운딩 박스 그리기"""
    detected_parts = set()
    for obj in objects:
        cls = obj["class"]
        score = obj["score"]
        box = obj["box"]
        color = colors.get(cls, (255, 255, 255))
        start_point = (int(box[0]), int(box[1]))
        end_point = (int(box[2]), int(box[3]))
        cv2.rectangle(frame, start_point, end_point, color, 2)
        text = f"{cls} ({score:.2f})"
        text_position = (int(box[0]), int(box[1]) - 10)
        cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        detected_parts.add(cls)
    return frame, detected_parts


def crop_img(img, size_dict):
    """지정된 크기로 이미지 크롭"""
    x = size_dict["x"]
    y = size_dict["y"]
    w = size_dict["width"]
    h = size_dict["height"]
    return img[y:y + h, x:x + w]


def sharpen_with_gaussian_blur(img):
    """Gaussian Blur를 이용해 이미지 샤프닝"""
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    sharpened = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
    return sharpened


@app.route('/')
def index():
    """메인 페이지"""
    return render_template('index.html', status_message=status_message)


@app.route('/video_feed')
def video_feed():
    """웹캠 피드 스트리밍"""
    def generate_frames():
        global status_message
        crop_info = {"x": 150, "y": 20, "width": 430, "height": 390}

        while True:
            # 아두이노로부터 데이터 읽기
            data = ser.read()
            if data == b"0":  # 아두이노 신호가 "0"일 때
                detected_parts_set = set()
                hole_valid_frame_found = False

                # 이전 이미지 데이터 초기화
                missing_parts = []
                detected_parts_set.clear()
                hole_valid_frame_found = False

                # 먼저 5 프레임을 객체 인식 없이 전송
                for i in range(5):
                    success, frame = camera.read()
                    if not success:
                        break

                    cropped_frame = crop_img(frame, crop_info)
                    ret, buffer = cv2.imencode(".jpg", cropped_frame)
                    frame = buffer.tobytes()

                    # 원본 이미지를 전송
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

                # 이후 15 프레임은 객체 인식 수행
                total_frames = 15  # 총 프레임 수
                for i in range(total_frames):
                    success, frame = camera.read()
                    if not success:
                        break

                    cropped_frame = crop_img(frame, crop_info)
                    cropped_frame = sharpen_with_gaussian_blur(cropped_frame)

                    result = inference_request(cropped_frame)
                    if result and "objects" in result:
                        if validate_hole_per_frame(result["objects"]):
                            hole_valid_frame_found = True

                        annotated_frame, detected_parts = draw_bounding_boxes(cropped_frame, result["objects"])
                        detected_parts_set.update(detected_parts)
                    else:
                        annotated_frame = cropped_frame

                    # 프레임 전송
                    ret, buffer = cv2.imencode(".jpg", annotated_frame)
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

                # 최종 결과 메시지 업데이트
                if hole_valid_frame_found:
                    detected_parts_set.add("Hole")
                else:
                    missing_parts.append("Hole")

                for part, count in required_parts.items():
                    if part not in detected_parts_set:
                        missing_parts.append(part)

                if missing_parts:
                    status_message = f"Missing parts detected: {', '.join(missing_parts)}"
                else:
                    status_message = "All required parts detected."

                # 아두이노 신호 전송
                ser.write(b"1")
                time.sleep(3)
                ser.write(b"0")

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')



@app.route('/status')
def status():
    """상태 메시지를 반환"""
    global status_message
    return {"message": status_message}


def validate_hole_per_frame(detected_objects, required_hole_count=4):
    """단일 프레임의 'Hole' 개수 확인"""
    hole_count = sum(1 for obj in detected_objects if obj["class"] == "Hole")
    return hole_count >= required_hole_count


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)



https://suite-endpoint-api-apne2.superb-ai.com/endpoints/cca5f8cc-3224-4f79-b712-1fa3c60c5bf8/inference