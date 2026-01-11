import cv2
import numpy as np

def main():
    # Windows 안정: 0번 카메라 + DSHOW
    camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    if not camera.isOpened():
        print("Camera open failed. Try index 1 or check camera permission.")
        return

    # 창을 늘리면 영상도 같이 늘어나게
    cv2.namedWindow('normal', cv2.WINDOW_NORMAL)
    cv2.namedWindow('crop', cv2.WINDOW_NORMAL)
    cv2.namedWindow('mask', cv2.WINDOW_NORMAL)

    # 초기 창 크기 (원하면 값 바꾸세요)
    cv2.resizeWindow('normal', 640, 480)
    cv2.resizeWindow('crop', 640, 240)
    cv2.resizeWindow('mask', 640, 240)

    while True:
        ret, frame = camera.read()
        if not ret or frame is None:
            print("Frame read failed.")
            break

        frame = cv2.flip(frame, -1)
        cv2.imshow('normal', frame)

        # ROI (하단)
        crop_img = frame[60:120, 0:160]
        h, w = crop_img.shape[:2]

        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 임계값(조명에 민감) - 원래 값 유지
        _, thresh = cv2.threshold(blur, 123, 255, cv2.THRESH_BINARY_INV)

        # 노이즈 제거
        mask = cv2.erode(thresh, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask', mask)

        # 윤곽선 찾기 (OpenCV 버전 차이 흡수)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = cnts[0] if len(cnts) == 2 else cnts[1]

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # 너무 작은 잡음 contour는 무시 (필요시 값 조정)
            if area > 50:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 중심선, 윤곽선 표시
                    cv2.line(crop_img, (cx, 0), (cx, h - 1), (255, 0, 0), 1)
                    cv2.line(crop_img, (0, cy), (w - 1, cy), (255, 0, 0), 1)
                    cv2.drawContours(crop_img, [c], -1, (0, 255, 0), 1)

                    # ✅ 원래 하던 출력 복구
                    print(cx)

        cv2.imshow('crop', crop_img)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
