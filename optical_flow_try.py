import cv2
import numpy as np
import matplotlib.pyplot as plt

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Parameters for Shi-Tomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# Load consecutive drone images (grayscale for simplicity)
def load_images(image_paths):
    images = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]
    return images

# Estimate optical flow between two frames
def estimate_flow(prev_frame, next_frame):
    # Detect good features to track (corners)
    prev_points = cv2.goodFeaturesToTrack(prev_frame, mask=None, **feature_params)

    # Calculate optical flow using Lucas-Kanade method
    next_points, status, err = cv2.calcOpticalFlowPyrLK(prev_frame, next_frame, prev_points, None, **lk_params)

    # Select good points where flow was found
    good_prev = prev_points[status == 1]
    good_next = next_points[status == 1]

    return good_prev, good_next

# Visualize the flow vectors
def visualize_flow(prev_frame, good_prev, good_next):
    plt.figure(figsize=(8, 8))
    plt.imshow(prev_frame, cmap='gray')
    for i, (prev, next) in enumerate(zip(good_prev, good_next)):
        x1, y1 = prev.ravel()
        x2, y2 = next.ravel()
        plt.arrow(x1, y1, x2 - x1, y2 - y1, color='r', head_width=3)
    plt.title('Optical Flow Vectors')
    plt.show()

# Estimate global motion of the drone
def estimate_global_motion(good_prev, good_next):
    motion_vectors = good_next - good_prev
    mean_motion = np.mean(motion_vectors, axis=0)
    return mean_motion

# Identify stationary objects (outliers in flow)
def identify_stationary_objects(good_prev, good_next, mean_motion, threshold=2.0):
    motion_vectors = good_next - good_prev
    distances = np.linalg.norm(motion_vectors - mean_motion, axis=1)
    stationary_points = good_prev[distances > threshold]
    return stationary_points

# Visualize stationary objects
def visualize_stationary_objects(frame, stationary_points):
    plt.figure(figsize=(8, 8))
    plt.imshow(frame, cmap='gray')
    for point in stationary_points:
        x, y = point.ravel()
        plt.scatter(x, y, c='b', s=40, marker='x')
    plt.title('Stationary Objects')
    plt.show()

# Main function to run the pipeline
def main(image_paths):
    images = load_images(image_paths)

    for i in range(len(images) - 1):
        prev_frame = images[i]
        next_frame = images[i + 1]

        # Estimate optical flow
        good_prev, good_next = estimate_flow(prev_frame, next_frame)

        # Visualize flow vectors
        visualize_flow(prev_frame, good_prev, good_next)

        # Estimate global motion
        mean_motion = estimate_global_motion(good_prev, good_next)

        # Identify stationary objects
        stationary_points = identify_stationary_objects(good_prev, good_next, mean_motion)

        # Visualize stationary objects
        visualize_stationary_objects(prev_frame, stationary_points)

# Example usage with dummy image paths
# image_paths = ['frame1.jpg', 'frame2.jpg', 'frame3.jpg']
# main(image_paths)


# This is a test to see that push works
