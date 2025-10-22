import cv2
import camera
import time

# Flag to show GUI
showGUI = True

# Initialize camera for MacBook
cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)

try:
    if showGUI:
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

    while True:
        # Capture next frame
        colour = cam.get_next_frame()
        
        if colour is None:
            print("No frame received from camera!")
            break

        # Show frame
        if showGUI:
            cv2.imshow(WIN_RF1, colour)

        # Quit on 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cv2.destroyAllWindows()
    cam.terminateCaptureThread()
