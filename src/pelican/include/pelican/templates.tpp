#ifndef _TEMPLATES_TPP
#define _TEMPLATES_TPP

template<typename T> void resetSharedPointer(std::shared_ptr<T>& ptr) {
    ptr.reset();
}


#endif
