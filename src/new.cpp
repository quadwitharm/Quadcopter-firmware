void *malloc(size_t size){}
void free(void *ptr){}
void* operator new(std::size_t size) {
    return malloc(size);
}
 
void* operator new[](std::size_t size) {
    return malloc(size);
}
 
void operator delete(void* ptr) {
    free(ptr);
}
 
void operator delete[](void* ptr) {
    free(ptr);
}
