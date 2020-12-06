#ifndef _FASTSTACK_H_
#define _FASTSTACK_H_

#include <iostream>
#include <memory>
#include <exception>
using namespace std;


/**
*
* @brief fast implementation STACK container. (via pre-allocation)(Template)
* @details fast implementation (template) STACK container. (via pre-allocation)
* @author Changhyeon Kim
* @date 2020-09-14
*
*/
template <typename T>
class FastStack {// stack only for 'node'
public:
    int sz;
    int MAX_SIZE;
    T* mem; // Point array (dynamically allocated)

            /**
            *
            * @brief Constructor for FastStack data structure.
            * @details Constructor for FastStack data structure.
            Main difference from the standard 'stack' is that memories for stack are pre-allocated in our implementations.
            * @return none.
            */
    FastStack() {
        MAX_SIZE = 65536; // stack needs no big size.
        mem = (T*)malloc(sizeof(T)*MAX_SIZE); // 8 * max_size
        sz = 0; // anything at first.
                // cout << "FASTSTACK init" << endl;
    };

    /**
    *
    * @brief destruct the FastStack and free the pre-allocated memories
    * @details destruct the FastStack and free the pre-allocated memories
    * @return none.
    */
    ~FastStack() { free(mem); };// free the stack memory // malloc -> free };

                                /**
                                *
                                * @brief Push a element onto the top of the stack.
                                * @details Push a element onto the top of the stack.
                                * @return none.
                                */
    void push(T& value_) { // increase stack size 1 when a new node is inserted.
        if (!isFull()) {
            *(mem + sz) = value_;
            ++sz;
        }
        else throw std::runtime_error("[ERROR]: FastStack is full.\n");
    };

    /**
    *
    * @brief Pop a element from the top of the stack.
    * @details Pop a element from the top of the stack.
    * @return none.
    */
    void pop() {
        if (!empty()) --sz;
        else throw std::runtime_error("[ERROR]: FastStack is empty.\n");
    };

    /**
    *
    * @brief Return the element in the top of the stack.
    * @details Return the element in the top of the stack.
    * @return Element of the top <template T>.
    */
    T top() {
        if (!empty()) return *(mem + (sz - 1));
        else return 0;
    };

    /**
    *
    * @brief Return true when stack is empty.
    * @details Return true when stack is empty.
    * @return true: empty, false: not empty.
    */
    bool empty() {
        if (sz < 1) return true;
        else return false;
    };

    /**
    *
    * @brief Return true when stack is 'FULL' (not just existing!).
    * @details Return true when stack is 'FULL' (not just existing!).
    * @return true: FULL, false: not full.
    */
    bool isFull() { return (sz == MAX_SIZE); };

    /**
    *
    * @brief clear the stack. (not free memories, but re-index to zero.)
    * @details clear the stack. (not free memories, but re-index to zero.)
    * @return none.
    */
    void clear() { sz = 0; };
};


// nodežžÀ» ÀúÀåÇÏŽÂ stackÀž·ÎŽÙ°¡ žžµéÀÚ.
template <typename T>
class PointerStack {
public:
    int size;
    int MAX_SIZE;
    int total_access;
    T** values;
    PointerStack() {
        MAX_SIZE = 65536;
        values = (T**)malloc(sizeof(T*)*MAX_SIZE); // 8 * max_size
        size = 0;
        total_access = 0; 
    }
    ~PointerStack() { delete[] values; }
    void push(T* value_) { 
        if (!isFull()) {
            *(values + size) = value_;
            ++size;
        }
        else printf("[ERROR]: NodeStack is full.\n");
    }
    void pop() {
        if (!empty()) --size;
        else printf("[ERROR]: NodeStack is empty.\n");
    }
    T* top() {
        if (!empty()) return *(values + (size - 1));
        else return nullptr;
    }
    T** root() {
        if (!empty()) return values;
        else return nullptr;
    }
    bool empty() {
        if (size < 1) return true;
        else return false;
    }
    bool isFull() { return (size == MAX_SIZE); }
    void clear() { size = 0; }

    /**
    *
    * @brief delete specific element if it is in the stack.
    * @details delete specific element if it is in the stack.
    * @return bool: true: delete, false: there is no such element.
    */
    bool eraseElem(const T* value_) {
        if (!empty()) {
            for (int i = this->size - 1; i > 0; i--) {
                if (*(values + i) == value_) {
                    if (i == this->size - 1)--this->size;
                    else {
                        for (int j = i; j < this->size - 1; j++)
                            *(values + i) = *(values + i + 1);
                    }
                }
                else {
                    cout << "[STACK WARNING]: There is no such element!\n";
                }
            }
        }
        else {
            throw std::runtime_error("Fail to stack erase!: empty stack!\n");
        }
    };

    void showAll() {
        for (int i = 0; i < size; i++) {
            cout << "[" << i << "] element: " << *(values + i) << "\n";
        }
    }
};

#endif