#ifndef __OBJECTS_H__
#define __OBJECTS_H__
// #include "Headers/Common.hpp"
// #include "Headers/Utils.hpp"
#include "Utils/Objects.hpp"
// #include "Headers/Publishers.hpp"
// #include "Headers/PointClouds.hpp"
// #include "Headers/Accumulator.hpp"
// #include "Headers/Compensator.hpp"
// #include "Headers/Localizator.hpp"
// #include "Headers/Mapper.hpp"
#include "Modules/Config.hpp"

#endif

// extern struct Params Config;

template class Buffer<IMU>;

// class Buffer {
    // public:
        template <typename ContentType>
        Buffer<ContentType>::Buffer() {}

        template <typename ContentType>
        void Buffer<ContentType>::push(const ContentType& cnt) {
            this->content.push_front(cnt);
        }
