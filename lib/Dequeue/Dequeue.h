template<typename T> 
class Dequeue {
    public:
        explicit Dequeue(uint16_t maxSize) {
            _values = (T *) malloc(sizeof(T) * maxSize);
            front = -1;
            rear = 0;
            size = maxSize;
        }
        ~Dequeue() {
            free(_values);
        }

        bool add(T e);

        bool push();

        T* peek();        
        
        T* peekLast();

        T* remove();

        T* pop();

        uint16_t size();

        bool isEmpty();
        
        bool isFull();

    protected:
        T* _values;
        int _size;
        int _front;
        int _rear;
}