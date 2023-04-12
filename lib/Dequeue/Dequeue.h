template<typename T> 
class Dequeue {
    public:
        explicit Dequeue(uint16_t maxSize) {
            _values = (T *) malloc(sizeof(T) * maxSize);
        }
        ~Dequeue() {
            free(_values);
        }

        boolean add(T e);

        T* peek();        
        
        T* peekLast();

        T* push();

        T* remove();

        T* pop();

        uint16_t size();

        void clear();

    protected:
        T* _values;
}