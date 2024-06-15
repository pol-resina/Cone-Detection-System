#ifndef BUFFER_HPP
#define BUFFER_HPP

// #include "Utils/Common.hpp"
// #include "Modules/Config.hpp"
// #include "Utils/Utils.hpp"

// #include "Objects/RotTransl.hpp"
#include "Objects/Point.hpp"
#include "Objects/IMU.hpp"
#include "Objects/State.hpp"

#include <deque>

template <typename ContentType>
class Buffer {
    public:
        std::deque<ContentType> content;
        Buffer();

        void push(const ContentType& cnt);
        void pop_front();
        void pop_back();    
        ContentType front();
        ContentType back();
        bool empty();
        int size();
        void clear();
        void clear(TimeType t);
};

#endif