/**
 * @file adde.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-06
 * @version 2019-05-10
 * 
 * @remarks Implementations can be found in ../Arduino.cpp
 */
#ifndef __ADDE_HPP__
#define __ADDE_HPP__

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <wchar.h>
#include "tchar.h"

#ifdef __cplusplus
#include "../arduino/Meta.hpp"
#include "../arduino/Pre.hpp"


namespace adde {


/** @internal structures to handle strong typed values */
template <typename T>
struct _V {
	typedef _V<T> base;
	typedef T type;
	T value;
	explicit _V(): value() {}
	explicit _V(const T & val): value(val) {}
	operator T() const { return this->value; }
	int printTo(FILE * fd) const;
};

template <> struct _V<void> { typedef void type; int printTo(FILE * /* fd */) const { return 0; } };

template <> int _V<bool>::printTo(FILE * fd) const;
template <> int _V<signed char>::printTo(FILE * fd) const;
template <> int _V<unsigned char>::printTo(FILE * fd) const;
template <> int _V<signed short>::printTo(FILE * fd) const;
template <> int _V<unsigned short>::printTo(FILE * fd) const;
template <> int _V<signed int>::printTo(FILE * fd) const;
template <> int _V<unsigned int>::printTo(FILE * fd) const;
template <> int _V<signed long>::printTo(FILE * fd) const;
template <> int _V<unsigned long>::printTo(FILE * fd) const;
template <> int _V<signed long long>::printTo(FILE * fd) const;
template <> int _V<unsigned long long>::printTo(FILE * fd) const;
template <> int _V<float>::printTo(FILE * fd) const;
template <> int _V<double>::printTo(FILE * fd) const;
template <> int _V<long double>::printTo(FILE * fd) const;
template <> int _V<void *>::printTo(FILE * fd) const;
template <> int _V<wchar_t>::printTo(FILE * fd) const;

struct _void : _V<void> { using _V::_V; static size_t size(); using _V::printTo; };
struct _bool : _V<bool> { using _V::_V; static size_t size(); using _V::printTo; };
struct _signed_char : _V<signed char> { using _V::_V; static size_t size(); using _V::printTo; };
struct _unsigned_char : _V<unsigned char> { using _V::_V; static size_t size(); using _V::printTo; };
struct _signed_short : _V<signed short> { using _V::_V; static size_t size(); using _V::printTo; };
struct _unsigned_short : _V<unsigned short> { using _V::_V; static size_t size(); using _V::printTo; };
struct _signed_int : _V<signed int> { using _V::_V; static size_t size(); using _V::printTo; };
struct _unsigned_int : _V<unsigned int> { using _V::_V; static size_t size(); using _V::printTo; };
struct _signed_long : _V<signed long> { using _V::_V; static size_t size(); using _V::printTo; };
struct _unsigned_long : _V<unsigned long> { using _V::_V; static size_t size(); using _V::printTo; };
struct _signed_long_long : _V<signed long long> { using _V::_V; static size_t size(); using _V::printTo; };
struct _unsigned_long_long : _V<unsigned long long> { using _V::_V; static size_t size(); using _V::printTo; };
struct _float : _V<float> { using _V::_V; static size_t size(); using _V::printTo; };
struct _double : _V<double> { using _V::_V; static size_t size(); using _V::printTo; };
struct _long_double : _V<long double> { using _V::_V; static size_t size(); using _V::printTo; };
struct _void_ptr : _V<void *> { using _V::_V; static size_t size(); using _V::printTo; };
struct _ptrdiff_t : _V<ptrdiff_t> { using _V::_V; static size_t size(); using _V::printTo; };
struct _wchar_t : _V<wchar_t> { using _V::_V; static size_t size(); using _V::printTo; };
struct _size_t : _V<size_t> { using _V::_V; static size_t size(); using _V::printTo; };
struct _int8_t : _V<int8_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _uint8_t : _V<uint8_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _int16_t : _V<int16_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _uint16_t : _V<uint16_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _int32_t : _V<int32_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _uint32_t : _V<uint32_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _int64_t : _V<int64_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };
struct _uint64_t : _V<uint64_t> { using _V::_V; static size_t size() { return sizeof(type); } using _V::printTo; };

typedef typename conditional<is_signed<char>::value, _signed_char, _unsigned_char>::type _char;
typedef typename conditional<is_signed<short>::value, _signed_short, _unsigned_short>::type _short;
typedef typename conditional<is_signed<int>::value, _signed_int, _unsigned_int>::type _int;
typedef typename conditional<is_signed<long>::value, _signed_long, _unsigned_long>::type _long;
typedef typename conditional<is_signed<long long>::value, _signed_long_long, _unsigned_long_long>::type _long_long;


template <typename T> struct _IsV : is_base_of<_V<typename T::type>, T> {};
template <typename T> struct _IsV<_V<T>> : true_type {};


/** @internal class to handle reference counting */
void * _createRcLock();
void _destroyRcLock(void * lock);
void _lockRcLock(void * lock);
void _unlockRcLock(void * lock);


template <typename T>
class _RC {
private:
    template <typename U> friend class _RC;
	struct Data {
		size_t count;
		void * lock;
		T * content;
		
		explicit Data(T * c = NULL):
			count(1),
			lock(::adde::_createRcLock()),
			content(c)
		{}
		
		Data(const Data & o) = delete;
		
		~Data() {
			::adde::_destroyRcLock(this->lock);
			delete content;
		}
		
		Data & operator= (const Data & o) = delete;
	};
	
	Data * data;
public:
	explicit _RC(T * d = NULL):
		data((d != NULL) ? new Data(d) : NULL)
	{}
	
	template <typename U, typename = enable_if<is_convertible<U, T>::value>>
	explicit _RC(_RC<U> & o):
		data(reinterpret_cast<typename _RC<T>::Data *>(o.data))
	{
		this->increase();
	}
	
	_RC(const _RC & o):
		data(o.data)
	{
		this->increase();
	}
	
	_RC(_RC && o):
		data(o.data)
	{
		o.decrease();
		o.data = NULL;
	}
	
	~_RC() {
		this->decrease();
	}
	
	_RC & operator= (const T * d) {
		this->decrease();
		this->data = new Data(d);
		return *this;
	}
	
	_RC & operator= (const _RC & o) {
		if (this != &o) {
			this->decrease();
			this->data = o.data;
			this->increase();
		}
		return *this;
	}
	
	_RC & operator= (_RC && o) {
		if (this != &o) {
			this->decrease();
			this->data = o.data;
			this->increase();
			o.decrease();
			o.data = NULL;
		}
		return *this;
	}
	
	T & operator* () const {
		assert(this->data != NULL);
		return *(this->data->content);
	}
	
	T * operator-> () const {
		return this->get();
	}
	
	T * get() const {
		if (this->data == NULL) return NULL;
		return this->data->content;
	}
	
	operator bool() const {
		return this->data != NULL && this->data->content != NULL;
	}
	
	void increase() {
		if (this->data == NULL) return;
		::adde::_lockRcLock(this->data->lock);
		this->data->count++;
		::adde::_unlockRcLock(this->data->lock);
	}
	
	/** @return true if reference count reached zero */
	bool decrease() {
		if (this->data == NULL) return true;
		::adde::_lockRcLock(this->data->lock);
		this->data->count--;
		const bool res = this->data->count == 0;
		::adde::_unlockRcLock(this->data->lock);
		if (res == true && this->data != NULL) {
			delete this->data;
			this->data = NULL;
		}
		return res;
	}
};


/** @internal allocates a new reference counting object */
template <typename T, typename ...Args>
_RC<T> _make_rc(Args ...args) {
	return _RC<T>(new T(args...));
}


/** @internal interface to handle remote call future value */
struct _FVI {
	unsigned long start;
	ssize_t timeout;
	int reference;
	bool success;
	bool valid;
	
	/**
	 * Constructor.
	 * 
	 * @param[in] aStart - start time as given from millis()
	 * @param[in] aTimeout - timeout in milliseconds
	 * @param[in] aReference - element reference value (usually a OpCode::Type value)
	 */
	explicit _FVI(const unsigned long aStart, const size_t aTimeout, const int aReference = 0):
		start(aStart),
		timeout(aTimeout),
		reference(aReference),
		success(false),
		valid(false)
	{}
	virtual ~_FVI();
	void waitForResult();
	
	/**
	 * Handling for call results of the given type.
	 * 
	 * @param[in,out] buf - buffer to read from
	 * @param[in,out] len - length of the buffer
	 * @return true on success, else false
	 */
	virtual bool set(uint8_t * & buf, size_t & len) = 0;
};


/** @internal class to handle remote call future value */
template <typename T>
struct _FV : public _FVI {
	typedef T type;
	T value;
	
	explicit _FV(const unsigned long aStart, const size_t aTimeout, const int aReference = 0):
		_FVI(aStart, aTimeout, aReference),
		value(T())
	{}
	
	virtual ~_FV() {}
	
	virtual bool set(uint8_t * & buf, size_t & len);
	
	operator typename T::type() {
		/* may trigger result timeout error which terminates the application */
		this->waitForResult();
		if (this->valid && this->success) return this->value;
		abort();
		return typename T::type();
	}
};


/** @internal class to handle remote call future value */
template <>
struct _FV<_void> : public _FVI {
	explicit _FV(const unsigned long aStart, const size_t aTimeout, const int aReference = 0):
		_FVI(aStart, aTimeout, aReference)
	{}
	
	virtual ~_FV() {}
	
	virtual bool set(uint8_t * & buf, size_t & len);
};


/** @internal class for remote call future handling */
template <typename T>
class _F {
private:
	_RC<_FV<T>> future;
public:
	explicit _F(const _RC<_FV<T>> & f):
		future(f)
	{}
	
	~_F() {}
	
	bool success() const {
		if ( ! this->future ) return false;
		this->future->waitForResult();
		return this->future->valid && this->future->success;
	}
	
	operator _RC<_FV<T>>() {
		return this->future;
	}
	
	operator typename T::type() {
		/* may trigger result timeout error which terminates the application */
		return static_cast<typename T::type>(*(this->future));
	}
};


/** @internal class for remote call future handling */
template <>
class _F<_void> {
private:
	_RC<_FV<_void>> future;
public:
	explicit _F(const _RC<_FV<_void>> & f):
		future(f)
	{}
	
	~_F() {}
	
	bool success() const {
		if ( ! this->future ) return false;
		this->future->waitForResult();
		return this->future->valid && this->future->success;
	}
	
	operator _RC<_FV<_void>>() {
		return this->future;
	}
};


} /* namespace adde */


#define ADDE_F(x) ::adde::_F<::adde::x>


#else /* !__cplusplus */
#define ADDE_F(x) PP_CAT(ADDE, x)()
#define ADDE_void() void
#define ADDE_bool() bool
#define ADDE_signed_char() signed char
#define ADDE_unsigned_char() unsigned char
#define ADDE_signed_short() signed short
#define ADDE_unsigned_short() unsigned short
#define ADDE_signed_int() signed int
#define ADDE_unsigned_int() unsigned int
#define ADDE_signed_long() signed long
#define ADDE_unsigned_long() unsigned long
#define ADDE_signed_long_long() signed long long
#define ADDE_unsigned_long_long() unsigned long long
#define ADDE_float() float
#define ADDE_double() double
#define ADDE_long_double() long double
#define ADDE_void_ptr() void *
#define ADDE_ptrdiff_t() ptrdiff_t
#define ADDE_wchar_t() wchar_t
#define ADDE_size_t() size_t
#define ADDE_int8_t() int8_t
#define ADDE_uint8_t() uint8_t
#define ADDE_int16_t() int16_t
#define ADDE_uint16_t() uint16_t
#define ADDE_int32_t() int32_t
#define ADDE_uint32_t() uint32_t
#define ADDE_int64_t() int64_t
#define ADDE_uint64_t() uint64_t
#endif /* __cplusplus */


#endif /* __HPP__ */
