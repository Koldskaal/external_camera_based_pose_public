import cv2


def create_image_points(original_image, zoom_factor=2.0):
    def click_event(event, x, y, flags, param):
        offset_x, offset_y, zoom_factor, zoomed_roi, original_coordinates, index = param
        if event == cv2.EVENT_LBUTTONDOWN:
            # Map the click coordinates to the original image coordinates
            orig_x = int(x / zoom_factor) + offset_x
            orig_y = int(y / zoom_factor) + offset_y
            original_coordinates[index] = (orig_x, orig_y)
            # print(f"Clicked at zoomed quadrant: ({x}, {y})")
            # print(f"Mapped to original image: ({orig_x}, {orig_y})")
            # Optionally, display the coordinates on the zoomed quadrant
            font = cv2.FONT_HERSHEY_SIMPLEX
            copy = zoomed_roi.copy()
            cv2.putText(
                copy,
                f"({orig_x},{orig_y})",
                (x - 20, y + 20),
                font,
                0.5,
                (0, 255, 0),
                1,
            )
            cv2.drawMarker(copy, (x, y), (9, 255, 9), cv2.MARKER_CROSS, 5, 2)
            cv2.imshow("Zoomed Quadrant", copy)

    # Get the dimensions of the original image
    height, width = original_image.shape[:2]

    quadrants = ["bottom-left", "bottom-right", "top-left", "top-right"]
    # quadrants = ["full", "full", "full", "full"]
    original_coordinates = []
    for quadrant in quadrants:
        original_coordinates.append((0, 0))
        # Determine the region of interest (ROI) based on the selected quadrant
        if quadrant == "top-left":
            roi = original_image[0 : height // 2, 0 : width // 2]
            offset_x, offset_y = 0, 0
        elif quadrant == "top-right":
            roi = original_image[0 : height // 2, width // 2 : width]
            offset_x, offset_y = width // 2, 0
        elif quadrant == "bottom-left":
            roi = original_image[height // 2 : height, 0 : width // 2]
            offset_x, offset_y = 0, height // 2
        elif quadrant == "bottom-right":
            roi = original_image[height // 2 : height, width // 2 : width]
            offset_x, offset_y = width // 2, height // 2
        elif quadrant == "full":
            roi = original_image[0:height, 0:width]
            offset_x, offset_y = 0, 0
        else:
            raise ValueError("Invalid quadrant specified.")

        # Calculate the new dimensions for the zoomed-in quadrant
        zoomed_width = int(roi.shape[1] * zoom_factor)
        zoomed_height = int(roi.shape[0] * zoom_factor)

        # Resize the ROI to achieve the zoom effect
        zoomed_roi = cv2.resize(
            roi, (zoomed_width, zoomed_height), interpolation=cv2.INTER_LINEAR
        )
        cv2.putText(
            zoomed_roi,
            f"Press any key to confirm",
            (40, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 0, 255),
            2,
        )

        # Display the zoomed quadrant in a window
        cv2.imshow("Zoomed Quadrant", zoomed_roi)

        # Set the mouse callback function to the window
        cv2.setMouseCallback(
            "Zoomed Quadrant",
            click_event,
            (offset_x, offset_y, zoom_factor, zoomed_roi, original_coordinates, -1),
        )
        # Wait until a key is pressed
        cv2.waitKey(0)

        # Destroy all windows
        cv2.destroyAllWindows()

        # # Save the original image coordinates to a file
        # with open('original_coordinates.txt', 'w') as file:
        #     for coord in original_coordinates:
        #         file.write(f"{coord[0]}, {coord[1]}\n")

    return original_coordinates


def create_vector_points(original_image, zoom_factor=1.0):
    def click_event_vec(event, x, y, flags, param):
        offset_x, offset_y, zoom_factor, zoomed_roi, original_coordinates = param
        if event == cv2.EVENT_LBUTTONDOWN:
            # Map the click coordinates to the original image coordinates
            orig_x = int(x / zoom_factor) + offset_x
            orig_y = int(y / zoom_factor) + offset_y
            if len(original_coordinates) < 2:
                original_coordinates.append((orig_x, orig_y))
            else:
                original_coordinates[0] = (
                    original_coordinates[1][0],
                    original_coordinates[1][1],
                )
                original_coordinates[1] = (orig_x, orig_y)

            # Optionally, display the coordinates on the zoomed quadrant
            copy = zoomed_roi.copy()
            temp = []
            for coord in original_coordinates:
                x = int((coord[0] - offset_x) * zoom_factor)
                y = int((coord[1] - offset_y) * zoom_factor)
                cv2.drawMarker(copy, (x, y), (9, 255, 9), cv2.MARKER_CROSS, 5, 2)
                temp.append((x, y))
            if len(original_coordinates) >= 2:
                cv2.arrowedLine(
                    copy,
                    temp[0],
                    temp[1],
                    (255, 255, 0),
                    thickness=2,
                    tipLength=0.05,
                )
            cv2.imshow("Hello", copy)

    original_coordinates = []
    # Calculate the new dimensions for the zoomed-in quadrant
    zoomed_width = int(original_image.shape[1] * zoom_factor)
    zoomed_height = int(original_image.shape[0] * zoom_factor)

    # Resize the ROI to achieve the zoom effect
    zoomed_roi = cv2.resize(
        original_image,
        (zoomed_width, zoomed_height),
        interpolation=cv2.INTER_LINEAR,
    )
    cv2.putText(
        zoomed_roi,
        f"Press any key to confirm",
        (40, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 0, 255),
        2,
    )

    copy = zoomed_roi.copy()

    # Display the zoomed quadrant in a window
    cv2.imshow("Hello", copy)

    # Set the mouse callback function to the window
    cv2.setMouseCallback(
        "Hello",
        click_event_vec,
        (0, 0, zoom_factor, zoomed_roi, original_coordinates),
    )
    # Wait until a key is pressed
    cv2.waitKey(0)

    # Destroy all windows
    cv2.destroyAllWindows()

    return original_coordinates


if __name__ == "__main__":
    # Load the original image
    image_path = "cropimg94.png"  # Replace with your image path
    original_image = cv2.imread(image_path)
    # print(create_image_points(original_image))
    print(create_vector_points(original_image, 5.0))
    print("done")
