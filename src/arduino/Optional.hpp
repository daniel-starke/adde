/**
 * @file Optional.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-24
 * @version 2019-05-04
 */
#ifndef __OPTIONAL_HPP__
#define __OPTIONAL_HPP__

#include "Meta.hpp"
#include "New.hpp"


/**
 * Helper class to dynamically create/destroy objects without dynamic heap allocation.
 * 
 * @tparam T - object type
 */
template <typename T>
class Optional {
private:
	char data[sizeof(T)];
	bool created;
public:
	inline explicit Optional():
		created(false)
	{}
	
	inline ~Optional() {
		this->reset();
	}
	
	template <typename ...Args>
	void set(Args ...args) {
		this->reset();
		new(this->data) T(args...);
		this->created = true;
	}
	
	void reset() {
		if ( this->created ) {
			(*this)->~T();
			this->created = false;
		}
	}
	
	inline T * operator-> () {
		if ( ! this->created ) return NULL;
		return reinterpret_cast<T *>(this->data);
	}
	
	inline const T * operator-> () const {
		if ( ! this->created ) return NULL;
		return reinterpret_cast<const T *>(this->data);
	}
	
	inline T & operator* () {
		return *reinterpret_cast<T *>(this->data);
	}
	
	inline const T & operator* () const {
		return *reinterpret_cast<const T *>(this->data);
	}
	
	inline operator bool() const {
		return this->created;
	}
};


#endif /* __OPTIONAL_HPP__ */
