#ifndef ITEM_SCANNER_H
#define ITEM_SCANNER_H

#include <string>
#include <sqlite3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

struct ItemInfo {
    std::string qr_code;
    std::string category;
    std::string size;
    std::string arrival_date;
    std::string destination;
    bool found;//if found the item
};

class ItemScanner {
public:
    ItemScanner(const std::string& db_path = "items.db");
    ~ItemScanner();

    bool initCamera(int camera_id = 0);

    ItemInfo scanFrame(const cv::Mat& frame);

    bool getFrame(cv::Mat& frame);

    void release();

private:
    sqlite3* db_;
    cv::VideoCapture cap_;
    cv::QRCodeDetector qr_decoder_;
    ItemInfo queryItem(const std::string& qr_data);
};
#endif // ITEM_SCANNER_H
