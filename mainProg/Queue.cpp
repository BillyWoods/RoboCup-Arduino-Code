#include "Queue.h"

#include <stdlib.h>
#include <string.h>

// used for the struct movement, so a version of this templated
//  class can be created for it
#include "Robot.h"

template <class T>
bool queue<T>::append(T* newElem)
{
    if (!newElem || !incrEnd()) 
        return false; // queue is full
    
    memcpy((void*)(elems + end), (void*) newElem, sizeof(T));
    return true;
}

template <class T>
T* queue<T>::getNext()
{
    T* nextElemPtr =  elems + start;
    if (!incrStart()) // start could not be incremented as queue is empty 
        return (T*) 0; // NULL pointer
    return nextElemPtr;
}

template <class T>
T* queue<T>::peek()
{
    T* nextElemPtr =  elems + start;
    if (queueLen < 1) // queue is empty
        return (T*) 0; // NULL pointer
    return nextElemPtr;
}

template <class T>
void queue<T>::reset()
{
    start = 0;
    end = -1;
    queueLen = 0;
}

template <class T>
queue<T>::queue(const int size)
    : maxQueueLen(size)
{ 
    // nothing 
}

template <class T>
void queue<T>::init()
{ 
    elems = (T*) malloc(maxQueueLen * sizeof(T));
    reset();
}

template <class T>
queue<T>::~queue()
{
    free(elems);
}

template <class T>
bool queue<T>::incrStart()
{
    // no more elements left
    if (queueLen < 1)
        return false;
    else
    {
        queueLen--;
        start = (start + 1) % maxQueueLen;
    }
}

template <class T>
bool queue<T>::incrEnd()
{
    // no more space left
    if (queueLen >= maxQueueLen)
        return false;
    else
    {
        queueLen++;
        end = (end + 1) % maxQueueLen;
        return true;
    }
}


template class queue<struct movement>;
template class queue<int>;
