#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/videoio/videoio_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define WIDTH 640
#define HEIGHT 480

// Define FlowVector structure
typedef struct {
    float x;
    float y;
    float magnitude;
    float angle;
} FlowVector;

// Compute optical flow
FlowVector* compute_optical_flow(IplImage* prev_gray, IplImage* gray) {
    CvSize size = cvGetSize(gray);
    CvMat* flow_x = cvCreateMat(size.height, size.width, CV_32FC1);
    CvMat* flow_y = cvCreateMat(size.height, size.width, CV_32FC1);

    // Calculate optical flow using Lucas-Kanade method
    cvCalcOpticalFlowLK(prev_gray, gray, cvSize(15, 15), flow_x, flow_y);

    // Allocate memory for flow vectors
    FlowVector* flow_vectors = (FlowVector*)malloc(size.width * size.height * sizeof(FlowVector));

    // Fill flow vectors array
    for (int y = 0; y < size.height; y++) {
        for (int x = 0; x < size.width; x++) {
            int idx = y * size.width + x;
            float fx = cvmGet(flow_x, y, x);
            float fy = cvmGet(flow_y, y, x);

            flow_vectors[idx].x = fx;
            flow_vectors[idx].y = fy;
            flow_vectors[idx].magnitude = sqrtf(fx * fx + fy * fy);
            flow_vectors[idx].angle = atan2f(fy, fx);
        }
    }

    // Clean up
    cvReleaseMat(&flow_x);
    cvReleaseMat(&flow_y);

    return flow_vectors;
}

// Create center-weighted mask
float* create_center_mask(int w, int h) {
    float* mask = (float*)malloc(w * h * sizeof(float));
    float center_x = w / 2.0f;
    float center_y = h / 2.0f;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float dx = x - center_x;
            float dy = y - center_y;
            float dist = dx * dx + dy * dy;
            float max_dim = (float)h > (float)w ? (float)h : (float)w;
            mask[y*w + x] = expf(-dist / max_dim);
        }
    }

    return mask;
}

int main() {
    // Initialize camera
    CvCapture* cap = cvCreateCameraCapture(0);
    if (!cap) {
        fprintf(stderr, "Failed to open camera\n");
        return -1;
    }

    // Set camera properties
    cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

    // Get initial frame
    IplImage* frame = cvQueryFrame(cap);
    if (!frame) {
        fprintf(stderr, "Failed to capture frame\n");
        cvReleaseCapture(&cap);
        return -1;
    }

    // Create grayscale images
    IplImage* prev_gray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
    IplImage* gray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

    // Convert first frame to grayscale
    cvCvtColor(frame, prev_gray, CV_BGR2GRAY);

    // Create window
    cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);

    // Main loop
    while (1) {
        // Capture new frame
        frame = cvQueryFrame(cap);
        if (!frame) break;

        // Convert to grayscale
        cvCvtColor(frame, gray, CV_BGR2GRAY);

        // Compute optical flow
        FlowVector* flow = compute_optical_flow(prev_gray, gray);

        // Visualize flow
        IplImage* flow_viz = cvCloneImage(frame);
        CvSize size = cvGetSize(gray);

        for (int y = 0; y < size.height; y += 10) {
            for (int x = 0; x < size.width; x += 10) {
                int idx = y * size.width + x;
                if (flow[idx].magnitude > 0.5) {
                    CvPoint p1 = cvPoint(x, y);
                    CvPoint p2 = cvPoint(x + (int)(flow[idx].x * 5), y + (int)(flow[idx].y * 5));
                    cvLine(flow_viz, p1, p2, CV_RGB(0, 255, 0), 1, CV_AA, 0);
                }
            }
        }

        // Show result
        cvShowImage("Optical Flow", flow_viz);

        // Free resources
        free(flow);
        cvReleaseImage(&flow_viz);

        // Copy current gray to previous
        cvCopy(gray, prev_gray, NULL);

        // Exit on 'q' key
        if (cvWaitKey(10) == 'q') break;
    }

    // Cleanup
    cvReleaseImage(&prev_gray);
    cvReleaseImage(&gray);
    cvReleaseCapture(&cap);
    cvDestroyWindow("Optical Flow");

    return 0;
}