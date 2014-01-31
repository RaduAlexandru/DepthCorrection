#include "FancyQueue.h"
#include <utility>      // std::pair


void FancyQueue::lock(){
    locked=true;

}

void FancyQueue::unlock(){
    locked=false;

}

bool FancyQueue::status(){
    return locked;
}

FancyQueue::FancyQueue(){

    locked=false;
}
