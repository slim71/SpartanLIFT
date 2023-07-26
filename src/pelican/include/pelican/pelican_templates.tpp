#ifndef _PELICAN_TEMPLATES_TPP
#define _PELICAN_TEMPLATES_TPP

template<typename T>
void PelicanUnit::resetSharedPointer(std::shared_ptr<T>& ptr) {
    ptr.reset();
}

#endif