#pragma once

#include <queue>
#include <utility>
#include <map>

typedef std::pair<float,float> pri_key_t;
//typedef std::pair<pri_key_t,Vertex*> queue_element_t;

/**
 * @brief The objects in the priority queue
 * 
 */
template <class node>
class QueueElement{
    public:
        QueueElement(pri_key_t key, node* vertex) : key(key),vertex(vertex) {}
        pri_key_t key;
        node* vertex;
};

/**
 * @brief Object used to compare QueueElement keys.
 * 
 */
template <class node>
class CompareKeys{
    public:
        bool operator()(QueueElement<node> elem1, QueueElement<node> elem2) {
            if((elem1.key.first>elem2.key.first) || (elem1.key.first == elem2.key.first && elem1.key.second>elem2.key.second)) return true;
            return false;
        }
};

/**
 * @brief Implementation of a priority queue with added functionality for element removal and arbitrary priority modification (increase or decrease)
 * 
 * @remark In the worst case, unless proper care is taken in the utilization, a lot of outdated data can remain in the priority queue.
 * 
 */
template <class node>
class FlexiblePriorityQueue{
    private:
        std::priority_queue<QueueElement<node>,std::vector<QueueElement<node>>,CompareKeys<node>> pq_;
        std::map<node*,pri_key_t> latest_key_;
        std::map<node*,bool> remove_;
    public:
        
        
        /**
         * @brief Get the Vertex pointer in the queue with the lowest key (thus highest priority).
         * 
         * @remark Queue cleanup is done in this function. If a element is outdated (has gotten lower priority or is marked for removal) is is discarded. Only relevant vertices are returned.
         * @warning If only outdated elements remain, the function returns nullptr, thus make sure to check for it before working with the pointer!
         * 
         * @return Vertex* 
         */
        node* top(){
            pri_key_t temp;
            node* temp2;
            QueueElement top(temp,temp2);
            while(pq_.size()>0){
                top = pq_.top();
                if(remove_.find(top.vertex)!=remove_.end() || top.key!=latest_key_[top.vertex]){
                    //Old version of queue element, the priority key has since been lowered (which means higher priority).
                    // De to the use of k_m in D* Extra Lite it can never become less priority (I think, not sure yet).

                    //Also, pop if element is marked for removal too.
                    pq_.pop();

                    //To be able to add the vertex again, clear the removal
                    remove_.erase(top.vertex);
                } else{
                    //There is no updated priority in the queue, so we can return.
                    return top.vertex;
                }
            }
            return nullptr; //This is returned if the last element in the priority queue was marked for removal or outdated.
        }
        /**
         * @brief Checks if the priority queue is empty.
         * 
         */
        bool empty(){
            return pq_.empty();
        }

        /**
         * @brief Returns the size of the priority queue
         * 
         * @return int The number of elements in the priority queue
         */
        int size(){
            return pq_.size();
        }

        /**
         * @brief Pushes a vertex (pointer) with a given key value to the priority queue, then thr queue automatically sorts the queue.
         * 
         * @remark If duplicate pushes accidentally are atttempted, the function rejects these attempts to ensure no duplicate entries in the priority queue.
         * 
         * @param s Vertex pointer
         * @param key Vertex key value, calculated by search algorithm
         */
        void push(node* s, pri_key_t key) {
            if(latest_key_.find(s)!=latest_key_.end() && key==latest_key_[s]) return;
            pq_.push(QueueElement(key,s));
            latest_key_[s]=key;
            remove_.erase(s);
        }

        /**
         * @brief Erases the top element from the priority queue.
         * 
         */
        void pop() {
            QueueElement<node> top = pq_.top();
            latest_key_.erase(top.vertex);
            pq_.pop();
        }

        /**
         * @brief Flags a vertex pointer for removal from the priority queue.
         * 
         * @param s The vertex beeing flagged.
         */
        void remove(node* s) {
            remove_[s]=true;
        }

        /**
         * @brief 
         * 
         */
        bool exists(node* s) {
            return (latest_key_.find(s)!=latest_key_.end() && remove_.find(s)==remove_.end());
        }

        void clear(){
            while(size()>0){
                pop();
            }
            latest_key_.clear();
            remove_.clear();
        }

        std::map<node*,pri_key_t>* getCurrentPriority(){
            return &latest_key_;
        }


};