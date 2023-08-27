#ifndef _TEMPLATES_TPP
#define _TEMPLATES_TPP

template<typename T> void Pelican::resetSharedPointer(std::shared_ptr<T>& ptr) {
    ptr.reset();
}

#endif
