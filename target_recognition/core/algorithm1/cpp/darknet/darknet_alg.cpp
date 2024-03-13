#include "darknet_alg.h"
#include "darknet.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

vector<detection_with_class> detect(image im, network *net, float thresh, float hier_thresh, float nms, int classes) {
    // 进行检测
    network_predict_image(net, im);
    int num_detections = 0;

    detection *detections = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, 0, 1, &num_detections);

    // NMS
    if (nms) {
        do_nms_sort(detections, num_detections, classes, nms);
    }

    // 将检测结果存储在 vector 中
    vector<detection_with_class> result;
    for (int i = 0; i < num_detections; ++i) {
        int best_class = -1;
        float best_prob = 0;
        for (int j = 0; j < classes; ++j) {
            if (detections[i].prob[j] > best_prob) {
                best_class = j;
                best_prob = detections[i].prob[j];
            }
        }
        if (best_class >= 0) {
            detection_with_class det_class = {detections[i], best_class};
            result.push_back(det_class);
        }
    }

    // 释放资源
    free_detections(detections, num_detections);

    return result;
}

image getCvImage(cv::Mat rgbImage)
{
    // 首先，转换 cv::Mat 到浮点表示
    cv::Mat floatImage;
    rgbImage.convertTo(floatImage, CV_32FC3, 1.0 / 255.0);

    // 创建 image 的结构体
    image img;
    img.w = floatImage.cols;
    img.h = floatImage.rows;
    img.c = floatImage.channels();
    img.data = (float*)malloc(img.w * img.h * img.c * sizeof(float));
    // 从 cv::Mat 复制数据到 image 结构体
    int step = floatImage.step1();
    for (int y = 0; y < img.h; ++y) {
        for (int x = 0; x < img.w; ++x) {
            for (int c = 0; c < img.c; ++c) {
                float pixel = floatImage.ptr<float>(y)[x * img.c + c];
                img.data[c * img.w * img.h + y * img.w + x] = pixel;
            }
        }
    }
    return img;
}

cv::Mat imageToMat(image img) {
    // 创建一个空的 cv::Mat
    cv::Mat mat(img.h, img.w, CV_32FC3);

    // 将数据从 image 结构复制到 cv::Mat
    for (int y = 0; y < img.h; ++y) {
        for (int x = 0; x < img.w; ++x) {
            for (int c = 0; c < img.c; ++c) {
                float pixel = img.data[c * img.w * img.h + y * img.w + x];
                mat.ptr<float>(y)[x * img.c + c] = pixel;
            }
        }
    }

    // 转换回标准的 8-bit 格式
    cv::Mat convertedMat;
    mat.convertTo(convertedMat, CV_8UC3, 255.0);
    return convertedMat;
}


// image load_image(char *filename, int w, int h, int c)
// {
// #ifdef OPENCV
//     image out = load_image_cv(filename, c);
// #else
//     image out = load_image_stb(filename, c);
// #endif  // OPENCV

//     if((h && w) && (h != out.h || w != out.w)){
//         image resized = resize_image(out, w, h);
//         free_image(out);
//         out = resized;
//     }
//     return out;
// }

// image load_image_color(char *filename, int w, int h)
// {
//     return load_image(filename, w, h, 3);
// }

// image resize_image(image im, int w, int h)
// {
//     image resized = make_image(w, h, im.c);   
//     image part = make_image(w, im.h, im.c);
//     int r, c, k;
//     float w_scale = (float)(im.w - 1) / (w - 1);
//     float h_scale = (float)(im.h - 1) / (h - 1);
//     for(k = 0; k < im.c; ++k){
//         for(r = 0; r < im.h; ++r){
//             for(c = 0; c < w; ++c){
//                 float val = 0;
//                 if(c == w-1 || im.w == 1){
//                     val = get_pixel(im, im.w-1, r, k);
//                 } else {
//                     float sx = c*w_scale;
//                     int ix = (int) sx;
//                     float dx = sx - ix;
//                     val = (1 - dx) * get_pixel(im, ix, r, k) + dx * get_pixel(im, ix+1, r, k);
//                 }
//                 set_pixel(part, c, r, k, val);
//             }
//         }
//     }
//     for(k = 0; k < im.c; ++k){
//         for(r = 0; r < h; ++r){
//             float sy = r*h_scale;
//             int iy = (int) sy;
//             float dy = sy - iy;
//             for(c = 0; c < w; ++c){
//                 float val = (1-dy) * get_pixel(part, c, iy, k);
//                 set_pixel(resized, c, r, k, val);
//             }
//             if(r == h-1 || im.h == 1) continue;
//             for(c = 0; c < w; ++c){
//                 float val = dy * get_pixel(part, c, iy+1, k);
//                 add_pixel(resized, c, r, k, val);
//             }
//         }
//     }

//     free_image(part);
//     return resized;
// }