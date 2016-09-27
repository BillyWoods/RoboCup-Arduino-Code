#ifndef QUEUE_H
#define QUEUE_H


/* 
    Generic queue object used for movement queue and other
      buffers, e.g. task queue

    This queue will perform deep copies of objects appended to
      it, so the original data need not be persistent in its
      original scope.
*/

template <class T>
class queue
{
    public:
        // returns true on success, false if queue full
        bool append(T* newElem); 
        // will return null pointer if no more elements left
        T* getNext();
        T* peek(); // same as getNext, but will not clear element off queue
        void reset();

        queue(const int size);
        void init();
        ~queue();

    private:
        const int maxQueueLen;
        int queueLen;
        int start, end;

        // return false when queue is empty
        bool incrStart();
        // returns false when queue is full
        bool incrEnd();

        T* elems;
};

#endif
