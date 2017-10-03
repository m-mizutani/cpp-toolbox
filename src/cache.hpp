/*
 * Copyright (c) 2016 Masayoshi Mizutani <mizutani@sfc.wide.ad.jp>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPPTB_SRC_CACHE_HPP__
#define CPPTB_SRC_CACHE_HPP__

#include <assert.h>

#include <map>
#include <vector>
#include <deque>
#include <string>
#include "./exception.hpp"
#include "./buffer.hpp"
#include "./hash.hpp"


namespace tb {

class HashKey : public Buffer {
 private:
  uint32_t hv_;

 public:
  HashKey() {
  }
  HashKey(const void* ptr, size_t len) : Buffer(ptr, len) {
    // this->hv_ = hash32(this->ptr(), this->len());
  }
  HashKey(const HashKey& obj) : Buffer(obj) {
    this->hv_ = obj.hv_;
  }
  ~HashKey() = default;

  inline uint32_t hv() const { return this->hv_; }

  bool operator==(const HashKey& obj) const {
    assert(this->finalized());
    assert(obj.finalized());

    if (this->hv_ == obj.hv_ && this->Buffer::operator==(obj)) {
      return true;
    } else {
      return false;
    }
  }

  void finalize() {
    this->hv_ = hash32(this->ptr(), this->len());
    this->Buffer::finalize();
  }
};

template <typename T>
class LruHash {
 public:

  class Node {
   private:
    T data_;
    const HashKey key_;
    Node *next_, *prev_;  // double linked list for Bucket
    Node *link_;          // single linked list for TimeSlot
    uint64_t update_;
    uint64_t tick_;
    uint32_t hv_;
    bool active_;

   public:
    Node() : next_(nullptr), prev_(nullptr), link_(nullptr),
             update_(0), tick_(0), active_(true) {
    }
    Node(T data, const HashKey& key, uint64_t tick)
        : data_(data), key_(key), next_(nullptr), prev_(nullptr),
          link_(nullptr), update_(0), tick_(tick), active_(true) {
    }
    virtual ~Node() {}

    // Read only methods
    T data() const { return this->data_; }
    virtual bool is_null() const { return (this->key_.len() == 0); }
    uint64_t tick() const { return this->tick_; }
    uint64_t update() const { return this->update_; }
    bool active() const { return this->active_; }
    const HashKey& key() const { return this->key_; }

    // Updatable methods
    void set_update(uint64_t curr_tick) {
      this->update_ = curr_tick;
    }
    void deactivate() {
      this->active_ = false;
    }
    void attach(Node *node) {
      Node *prev = this;
      Node *next = this->next_;

      node->prev_ = prev;
      node->next_ = next;
      prev->next_ = node;
      if (next) {
        next->prev_ = node;
      }
    }
    void detach() {
      Node *next = this->next_;
      Node *prev = this->prev_;
      if (next) {
        next->prev_ = prev;
      }
      if (prev) {
        prev->next_ = next;
      }
      this->next_ = this->prev_ = nullptr;
    }
    void push_link(Node * node) {
      Node * next = this->link_;
      node->link_ = next;
      this->link_ = node;
    }
    Node* pop_link() {
      Node *node = this->link_;
      if (node) {
        Node *next = node->link_;
        this->link_ = next;
      }
      return node;
    }
    Node* pop_all() {
      Node * all = this->link_;
      this->link_ = nullptr;
      return all;
    }

    bool has_link() const {
      return (this->link_ != nullptr);
    }

    Node* search(const HashKey& key) {
      if (this->key_.finalized() && this->key_ == key) {
        return this;
      } else if (this->next_) {
        return this->next_->search(key);
      } else {
        return nullptr;
      }
    }
  };

 private:
  // ------------------------------------------
  class Bucket {
   private:
    Node root_;

   public:
    Bucket() = default;
    ~Bucket() = default;
    void attach(Node *node) {
      this->root_.attach(node);
    }
    Node* search(const HashKey& key) {
      return this->root_.search(key);
    }
  };

  // ------------------------------------------
  class TimeSlot {
   private:
    Node root_;
   public:
    TimeSlot() = default;
    ~TimeSlot() = default;
    void push(Node *node) {
      this->root_.push_link(node);
    }
    Node* pop() {
      return this->root_.pop_link();
    }
  };

  // ------------------------------------------
  std::vector<TimeSlot> timeslot_;
  std::vector<Bucket> bucket_;
  uint64_t curr_tick_;
  Node exp_node_;
  static const size_t DEFAULT_BUCKET_SIZE = 1031;
  Node null_node_;
  bool auto_update_;

  void insert(Node* node, uint64_t tick) {
    size_t ptr = node->key().hv() % this->bucket_.size();
    this->bucket_[ptr].attach(node);

    size_t tp = (tick + this->curr_tick_) % this->timeslot_.size();
    this->timeslot_[tp].push(node);
  }

  Node* lookup(const HashKey& key) {
    size_t ptr = key.hv() % this->bucket_.size();
    return this->bucket_[ptr].search(key);
  }
  
  static inline uint32_t key2hv(const Buffer& key) {
    return hash32(key.ptr(), key.len());
  }
  
 public:
  explicit LruHash(size_t timeslot_size,
                   size_t bucket_size = DEFAULT_BUCKET_SIZE)
      : timeslot_(timeslot_size), bucket_(bucket_size), curr_tick_(0),
        auto_update_(true) {
    if (this->bucket_.size() == 0) {
      this->bucket_.resize(DEFAULT_BUCKET_SIZE);
    }
  }
  ~LruHash() {    
  }

  bool put(uint64_t tick, const HashKey& key, T data) {
    if (tick < 1) {
      return false;
    }
    if (tick >= this->timeslot_.size()) {
      return false;
    }

    Node *node = new Node(data, key, tick);
    node->set_update(this->curr_tick_);

    this->insert(node, tick);
    return true;
  }

  const Node& get(const HashKey& key) {
    // Updated if hitting cache
    Node* node = this->lookup(key);
    
    if (node == nullptr) {
      node = &(this->null_node_);
    } else {
      if (this->auto_update_) {
        node->set_update(this->curr_tick_);
      }
    }

    return *node;
  }

  bool remove(const HashKey& key) {
    // Updated if hitting cache
    Node* node = this->lookup(key);

    if (node == nullptr) {
      return false;
    } else {
      node->detach();
      node->deactivate();
      return true;
    }
  }
  
  bool has(const HashKey& key) {
    Node* node = this->lookup(key);
    return (node != nullptr && node->active());
  }

  bool update(const HashKey& key) {
    Node* node = this->lookup(key);
    if (node) {
      node->set_update(this->curr_tick_);
      return true;
    } else {
      return false;
    }
  }

  void step(uint64_t tick = 1) {   // progress tick
    const uint64_t base_tick = this->curr_tick_;
    this->curr_tick_ += tick;

    for (size_t i = 1; i <= tick; i++) {
      size_t tp = (base_tick + i) % this->timeslot_.size();

      Node *node;
      while (nullptr != (node = this->timeslot_[tp].pop())) {
        node->detach();

        if (node->active() == false) {
          delete node; // nothing to do
        } else if (node->update() + node->tick() > this->curr_tick_) {
          const uint64_t remain = node->update() + node->tick()
                                  - this->curr_tick_;
          // re-insert
          this->insert(node, remain);
        } else {
          // expired, move to pop list
          this->exp_node_.push_link(node);
        }
      }
    }
    return;
  }

  void wipe() {
    for (auto& slot : this->timeslot_) {
      Node *node;
      while (nullptr != (node = slot.pop())) {
        node->detach();
        this->exp_node_.push_link(node);
      }
    }
  }

  bool has_expired() const {
    return this->exp_node_.has_link();
  }

  T pop_expired() {
    Node* node = this->exp_node_.pop_link();
    if (node) {
      T data = node->data();
      delete node;
      return data;
    } else {
      throw Exception::NoDataError("no more expired data");
    }
  }  // pop expired node

  void set_auto_update(bool b) { this->auto_update_ = b; }
};

}  // namespace pm

#endif  // CPPTB_SRC_CACHE_HPP__

