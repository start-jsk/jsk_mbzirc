// Copyright (C) 2016 by Krishneel Chaudhary @ JSK Lab, The University
// of Tokyo

#include <jsk_mbzirc_tasks/histogram_of_oriented_gradients.h>

HOGFeatureDescriptor::HOGFeatureDescriptor(
    const int cell_size, const int block_per_cell,
    const int n_bins, const float angle, const int num_threads) :
    CELL(cell_size),
    BLOCK(block_per_cell),
    ANGLE(angle),
    N_BINS(n_bins),
    num_threads_(num_threads_) {
    this->BINS_ANGLE = this->ANGLE / this->N_BINS;
}

void HOGFeatureDescriptor::histogramBinVoting(
    const float &angle, int &lower_index, int &higher_index) {
    float nearest_lower = FLT_MAX;
    float nearest_higher = FLT_MAX;
    lower_index = 0;
    higher_index = 0;
    for (int i = BINS_ANGLE/2; i < ANGLE; i += BINS_ANGLE) {
       float distance = abs(angle - i);
        if (i < angle) {
            if (distance < nearest_lower) {
                nearest_lower = distance;
                lower_index = i;
            }
        }
        /*
        else {
            if (distance < nearest_higher) {
                nearest_higher = distance;
                higher_index = i;
            }
        }
        */
    }
}

void HOGFeatureDescriptor::imageGradient(
    const cv::Mat &image, cv::Mat &hog_bins) {
    cv::Mat xsobel;
    cv::Mat ysobel;
    cv::Sobel(image, xsobel, CV_32F, 1, 0, 7);
    cv::Sobel(image, ysobel, CV_32F, 0, 1, 7);
    cv::Mat Imag;
    cv::Mat Iang;
    cv::cartToPolar(xsobel, ysobel, Imag, Iang, true);
    cv::add(Iang, cv::Scalar(ANGLE), Iang, Iang < 0);
    cv::add(Iang, cv::Scalar(-ANGLE), Iang, Iang >= ANGLE);
    cv::Mat orientation_histogram;
    for (int j = 0; j < image.rows; j += CELL) {
        for (int i = 0; i < image.cols; i += CELL) {
           cv::Rect rect = cv::Rect(i, j, CELL, CELL);
           if ((rect.x + rect.width <= image.cols) &&
               (rect.y + rect.height <= image.rows)) {
              cv::Mat bin = cv::Mat::zeros(1, N_BINS, CV_32F);
                for (int y = rect.y; y < (rect.y + rect.height); y++) {
                    for (int x = rect.x; x < (rect.x + rect.width); x++) {
                       float angle = static_cast<float>(Iang.at<float>(y, x));
                       int l_bin;
                       int h_bin;
                       this->histogramBinVoting(angle, l_bin, h_bin);
                       float l_ratio = 1.0f - (angle - l_bin)/BINS_ANGLE;
                       float h_ratio = 1.0f - l_ratio;
                       int l_index = (l_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                       // int h_index =
                       // (h_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                       int h_index = l_index + 1;
                       bin.at<float>(0, l_index) +=
                           (Iang.at<float>(y, x) * l_ratio);
                       bin.at<float>(0, h_index) +=
                           (Iang.at<float>(y, x) * h_ratio);
                    }
                }
                orientation_histogram.push_back(bin);
            }
        }
    }
    hog_bins = orientation_histogram.clone();
}

cv::Mat HOGFeatureDescriptor::blockGradient(
    const int col, const int row, const int stride, cv::Mat &bins) {
    cv::Mat block_hogMD = cv::Mat(cv::Size(N_BINS * BLOCK * BLOCK, 1), CV_32F);    
    int icounter = 0;
    for (int j = 0; j < BLOCK; j++) {
        for (int i = 0; i < BLOCK; i++) {
           int index = i + (j * stride) + col;
           for (int k = 0; k < N_BINS; k++) {
               block_hogMD.at<float>(0, icounter++) = bins.at<float>(index, k);
           }
       }
    }
    return block_hogMD;
}

void HOGFeatureDescriptor::getHOG(
    const cv::Mat &image, cv::Mat &bins, cv::Mat &featureMD) {
    const int stride = static_cast<int>(image.cols/CELL);
    int index = 0;
    for (int j = 0; j < image.rows - CELL; j += CELL) {
       for (int i = 0; i < image.cols - CELL; i += CELL) {
           cv::Mat hogMD = this->blockGradient(index, index, stride, bins);   
           // cv::normalize(hogMD, hogMD, 1, 0, CV_L2);
           cv::normalize(hogMD, hogMD, 1, 0, cv::NORM_L2);
           featureMD.push_back(hogMD);
           index++;
        }
    }
}

cv::Mat HOGFeatureDescriptor::computeHOG(
    const cv::Mat &img) {
    cv::Mat image = img.clone();
    if (image.type() != CV_8U) {
       cv::cvtColor(image, image, CV_BGR2GRAY);
    }
    cv::Mat bins;
    this->imageGradient(image, bins);    
    cv::Mat featureMD;
    getHOG(image, bins, featureMD);
    featureMD = featureMD.reshape(1, 1);
    return featureMD;
}

template<typename T>
T HOGFeatureDescriptor::computeHOGHistogramDistances(
    const cv::Mat &patch, std::vector<cv::Mat> &imageHOG,
    const int distance_type) {
    T sum = 0.0;
    T argMinDistance = FLT_MAX;
    for (int i = 0; i < imageHOG.size(); i++) {
       cv::Mat img_hog = imageHOG[i];
       T d = compareHist(patch, img_hog, distance_type);
        if (d < argMinDistance) {
            argMinDistance = static_cast<double>(d);
        }
    }
    sum = static_cast<T>(argMinDistance);
    return static_cast<T>(sum);
}

cv::Mat HOGFeatureDescriptor::visualizeHOG(
    cv::Mat& src, cv::Mat & descriptor_values, cv::Size win_size,
    int scale_factor, double viz_factor) {   
    cv::Mat visual_image;
    cv::Size cell_size = cv::Size(CELL, CELL);
    cv::resize(src, visual_image, cv::Size(src.cols*scale_factor,
                                           src.rows*scale_factor));
    int gradientBinSize = N_BINS;
    float radRangeForOneBin = M_PI/(float)gradientBinSize;
    int cells_in_x_dir = win_size.width / cell_size.width;
    int cells_in_y_dir = win_size.height / cell_size.height;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradient_strengths = new float**[cells_in_y_dir];
    int** cell_update_counter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++) {
        gradient_strengths[y] = new float*[cells_in_x_dir];
        cell_update_counter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++) {
            gradient_strengths[y][x] = new float[gradientBinSize];
            cell_update_counter[y][x] = 0;
            for (int bin=0; bin<gradientBinSize; bin++)
                gradient_strengths[y][x][bin] = 0.0;
        }
    }
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
    
    int descriptor_data_idx = 0;
    int cellx = 0;
    int celly = 0;

    for (int blockx=0; blockx<blocks_in_x_dir; blockx++) {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++) {
            for (int cellNr=0; cellNr<4; cellNr++) {
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3) {
                    cellx++;
                    celly++;
                }
                for (int bin=0; bin<gradientBinSize; bin++) {
                    float gradient_strength = descriptor_values.at<float>(
                        0, descriptor_data_idx);
                    descriptor_data_idx++;
                    gradient_strengths[celly][cellx][bin] += gradient_strength;
                }
                cell_update_counter[celly][cellx]++;
            }
        }
    }

    for (int celly=0; celly<cells_in_y_dir; celly++) {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++) {
            float NrUpdatesForThisCell = (float)cell_update_counter[celly][cellx];
            for (int bin=0; bin<gradientBinSize; bin++) {
                gradient_strengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }

    for (int celly=0; celly<cells_in_y_dir; celly++) {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++) {
            int drawX = cellx * cell_size.width;
            int drawY = celly * cell_size.height;
            int mx = drawX + cell_size.width/2;
            int my = drawY + cell_size.height/2;
            cv::rectangle(visual_image,
                          cv::Point(drawX*scale_factor,drawY*scale_factor),
                          cv::Point((drawX+cell_size.width)*scale_factor,
                                    (drawY+cell_size.height)*scale_factor),
                          CV_RGB(100,100,100),
                          1);
            for (int bin=0; bin<gradientBinSize; bin++) {
                float currentGradStrength = gradient_strengths[celly][cellx][bin];
                if (currentGradStrength==0)
                    continue;
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cell_size.width/2;
                float scale = viz_factor;
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
                cv::line(visual_image,
                     cv::Point(x1*scale_factor,y1*scale_factor),
                     cv::Point(x2*scale_factor,y2*scale_factor),
                         CV_RGB(0,0,255),
                         1);
            }
        }
    }
    for (int y=0; y<cells_in_y_dir; y++) {
        for (int x=0; x<cells_in_x_dir; x++) {
            delete[] gradient_strengths[y][x];            
        }
        delete[] gradient_strengths[y];
        delete[] cell_update_counter[y];
    }
    delete[] gradient_strengths;
    delete[] cell_update_counter;
    return visual_image;
}
