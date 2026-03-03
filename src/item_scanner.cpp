#include "item_scanner.h"
#include <iostream>

ItemScanner::ItemScanner(const std::string& db_path) {
    int rc = sqlite3_open(db_path.c_str(), &db_);
    if (rc != SQLITE_OK) {
        db_ = nullptr;
    }
}

ItemScanner::~ItemScanner() {
    release();
}

bool ItemScanner::initCamera(int camera_id) {
    cap_.open(camera_id);
    if (!cap_.isOpened()) {
        return false;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    return true;
}

bool ItemScanner::getFrame(cv::Mat& frame) {
    if (!cap_.isOpened()) return false;
    cap_ >> frame;
    return !frame.empty();
}

ItemInfo ItemScanner::scanFrame(const cv::Mat& frame) {
    ItemInfo info;
    info.found = false;
    if (!db_) {
        return info;
    }
    std::string qr_data = qr_decoder_.detectAndDecode(frame);
    if (qr_data.empty()) {
        return info;
    }
    info = queryItem(qr_data);
    return info;
}

ItemInfo ItemScanner::queryItem(const std::string& qr_data) {
    ItemInfo info;
    info.qr_code = qr_data;
    info.found = false;

    sqlite3_stmt* stmt;
    const char* sql = "SELECT category, size, arrival_date, destination FROM items WHERE qr_code = ?";

    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        return info;
    }

    sqlite3_bind_text(stmt, 1, qr_data.c_str(), -1, SQLITE_STATIC);

    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        info.found = true;
        info.category = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        info.size = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        info.arrival_date = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        info.destination = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        std::cout << "category=" << info.category
                  << ", size=" << info.size
                  << ", arrival_date=" << info.arrival_date
                  << ", destination=" << info.destination << std::endl;
    }
    sqlite3_finalize(stmt);
    return info;
}

void ItemScanner::release() {
    if (cap_.isOpened()) cap_.release();
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}
