#pragma once

#include "crtp_cpp/link/crtp_link.hpp" // Include your CrtpLink header
#include <memory> // For std::unique_ptr

class Logic {
public:
    // Use a unique_ptr to manage the CrtpLink object
    Logic(CrtpLink * crtp_link);
    virtual ~Logic() = default; // Important: Virtual destructor for ABC

protected:
    CrtpLink * link; // unique_ptr member
};