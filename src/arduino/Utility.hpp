/**
 * @file Utility.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-28
 * @version 2019-05-02
 */
#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "Arduino.h"
#include "Framing.hpp"
#include "Meta.hpp"


/** Frame reception callback arguments. */
struct FrameHandlerArgs {
	const uint8_t seq;
	const OpCode::Type op;
	uint8_t * buf;
	size_t len;
	
	/**
	 * Constructor.
	 * 
	 * @param[in] s - sequence number of the received frame
	 * @param[in] o - OP code of the received frame
	 * @param[in] b - buffer to the call arguments
	 * @param[in] l - length of the buffer
	 */
	explicit FrameHandlerArgs(const uint8_t s, const OpCode::Type o, uint8_t * b, size_t l):
		seq(s),
		op(o),
		buf(b),
		len(l)
	{}
};


/** Result of a remote function call called locally with the given framed argument list. */
template <typename T>
struct CallResult {
	bool success;
	T result;
};


/** Result of a remote function call called locally with the given framed argument list. */
template <>
struct CallResult<void> {
	bool success;
};


/**
 * Helper function to read back a given type from the passed buffer.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[out] out - variable to set
 * @return true on success, false on out-of-boundary error
 */
template <typename T, typename enable_if<is_arithmetic<T>::value>::type * = nullptr>
static bool readFrameValue(FrameHandlerArgs & hal, T & out) {
	if (sizeof(T) > hal.len) return false;
	if (sizeof(T) > 1) {
#if __FRAMING_HPP__IS_BIG_ENDIAN
		memcpy(reinterpret_cast<uint8_t *>(&out), hal.buf, sizeof(T));
#else /* reverse little endian to big endian */
		uint8_t * ptr = reinterpret_cast<uint8_t *>(&out) + sizeof(T) - 1;
		for (size_t i = 0; i < sizeof(T); i++) {
			*ptr-- = hal.buf[i];
		}
#endif
	} else {
		out = *hal.buf;
	}
	hal.buf += sizeof(T);
	hal.len = size_t(hal.len - sizeof(T));
	return true;
}


/**
 * Helper function to read back a given type from the passed buffer.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[out] out - variable to set
 * @return true on success, false on out-of-boundary error
 */
template <
	typename T,
	typename B = typename remove_pointer<T>::type,
	typename enable_if<is_pointer<T>::value && is_arithmetic<B>::value>::type * = nullptr
>
static bool readFrameValue(FrameHandlerArgs & hal, T & out) {
	if (sizeof(uint16_t) > hal.len) return false;
	uint16_t outLen;
	if ( ! readFrameValue(hal, outLen) ) return false;
	if (size_t(outLen) > hal.len) return false;
#if !__FRAMING_HPP__IS_BIG_ENDIAN
	/* reverse little endian to big endian */
	if (sizeof(B) > 1) {
		for (uint16_t i = 0; i < outLen; i++) {
			reverse(hal.buf + (i * sizeof(B)), hal.buf + ((i + 1) * sizeof(B)));
		}
	}
#endif
	out = hal.buf;
	hal.buf += outLen;
	hal.len = size_t(hal.len - outLen);
	return true;
}


/**
 * Helper function to terminate parameter pack expansion.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[out] args - tuple with arguments to fill
 * @return true on success, false on out-of-boundary error
 * @tparam i - argument index
 * @tparam Args - types of all function arguments
 */
template <size_t i = 0, typename ...Args>
static typename enable_if<i == sizeof...(Args), bool>::type extractArgs(FrameHandlerArgs &, tuple<Args...> &) {
	return true;
}


/**
 * Helper function to extract a single function argument from the given buffer. The buffer pointer
 * length variable are adjusted to extract the next argument on success.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[out] args - tuple with arguments to fill
 * @return true on success, false on out-of-boundary error
 * @tparam i - argument index
 * @tparam Args - types of all function arguments
 */
template <size_t i = 0, typename ...Args>
static typename enable_if<(i < sizeof...(Args)), bool>::type extractArgs(FrameHandlerArgs & hal, tuple<Args...> & args) {
	return readFrameValue(hal, get<i>(args)) && extractArgs<i + 1, Args...>(hal, args);
}


/* Template based index sequence storage and generator to unpack/iterate through tuples. */
template <size_t ...> struct ArgIndexSeq {};
template <size_t i, size_t ...rest> struct ArgIndexGen : ArgIndexGen<i - 1, i - 1, rest...> { };
template <size_t ...rest> struct ArgIndexGen<0, rest...> { typedef ArgIndexSeq<rest...> type; };


/**
 * Helper function to extract the function arguments from the given tuple and call the passed
 * function with those arguments passed. The return value of the function is returned.
 * 
 * @param[out] res - output variable for the function return value
 * @param[in] fn - function to call
 * @param[in] packedArgs - tuple with function arguments
 * @param[in] idx - tuple indices for all function arguments
 * @tparam i - argument indices
 * @tparam Fn - function type
 * @tparam T - tuple type
 * @tparam R - function return value type (not void)
 */
template <
	size_t ...i,
	typename Fn,
	typename T,
	typename R = typename return_type_of<typename remove_pointer<Fn>::type>::type,
	typename enable_if<!is_void<R>::value>::type * = nullptr
>
static void callUnpacked(CallResult<R> & res, Fn fn, const T & packedArgs, const ArgIndexSeq<i...>) {
	res.result = fn(get<i>(packedArgs)...);
}


/**
 * Helper function to extract the function arguments from the given tuple and call the passed
 * function with those arguments passed.
 * 
 * @param[out] res - output variable for the function return value
 * @param[in] fn - function to call
 * @param[in] packedArgs - tuple with function arguments
 * @param[in] idx - tuple indices for all function arguments
 * @tparam i - argument indices
 * @tparam Fn - function type
 * @tparam T - tuple type
 * @tparam R - function return value type (void)
 */
template <
	size_t ...i,
	typename Fn,
	typename T,
	typename R = typename return_type_of<typename remove_pointer<Fn>::type>::type,
	typename enable_if<is_void<R>::value>::type * = nullptr
>
static void callUnpacked(CallResult<R> &, Fn fn, const T & packedArgs, const ArgIndexSeq<i...>) {
	fn(get<i>(packedArgs)...);
}


/**
 * Helper function to extract the function arguments from the given tuple and call the passed
 * member function with those arguments passed. The return value of the function is returned.
 * 
 * @param[out] res - output variable for the function return value
 * @param[in] fn - function to call
 * @param[in,out] obj - object to use
 * @param[in] packedArgs - tuple with function arguments
 * @param[in] idx - tuple indices for all function arguments
 * @tparam i - argument indices
 * @tparam Fn - member function type
 * @tparam O - object class type
 * @tparam T - tuple type
 * @tparam R - function return value type (not void)
 */
template <
	size_t ...i,
	typename Fn,
	typename O,
	typename T,
	typename R = typename return_type_of<typename remove_pointer<Fn>::type>::type,
	typename enable_if<!is_void<R>::value>::type * = nullptr
>
static void callUnpacked(CallResult<R> & res, Fn fn, O & obj, const T & packedArgs, const ArgIndexSeq<i...>) {
	res.result = (obj.*fn)(get<i>(packedArgs)...);
}


/**
 * Helper function to extract the function arguments from the given tuple and call the passed
 * member function with those arguments passed.
 * 
 * @param[out] res - output variable for the function return value
 * @param[in] fn - function to call
 * @param[in,out] obj - object to use
 * @param[in] packedArgs - tuple with function arguments
 * @param[in] idx - tuple indices for all function arguments
 * @tparam i - argument indices
 * @tparam Fn - member function type
 * @tparam O - object class type
 * @tparam T - tuple type
 * @tparam R - function return value type (void)
 */
template <
	size_t ...i,
	typename Fn,
	typename O,
	typename T,
	typename R = typename return_type_of<typename remove_pointer<Fn>::type>::type,
	typename enable_if<is_void<R>::value>::type * = nullptr
>
static void callUnpacked(CallResult<R> & res, Fn fn, O & obj, const T & packedArgs, const ArgIndexSeq<i...>) {
	(obj.*fn)(get<i>(packedArgs)...);
}


/**
 * Extracts the arguments from the given buffer and calls the function passed with the extracted
 * arguments.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] fn - function to call
 * @remarks The given buffer may be modified for endian conversion.
 * @tparam R - function return value type
 * @tparam Args - argument types
 */
template <typename R, typename ...Args>
static CallResult<R> callWith(FrameHandlerArgs & hal, R(* fn)(Args...)) {
	CallResult<R> res;
	tuple<Args...> args;
	res.success = extractArgs(hal, args);
	if ( res.success ) callUnpacked(res, fn, args, typename ArgIndexGen<sizeof...(Args)>::type());
	return res;
}


/**
 * Extracts the arguments from the given buffer and calls the static member function passed with the
 * extracted arguments.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] fn - function to call
 * @param[in] obj - object to use (dummy)
 * @remarks The given buffer may be modified for endian conversion.
 * @tparam R - function return value type
 * @tparam C - class type
 * @tparam Args - argument types
 */
template <typename R, typename C, typename ...Args>
static CallResult<R> callWith(FrameHandlerArgs & hal, R(* fn)(Args...), C &) {
	return callWith(hal, fn);
}


/**
 * Extracts the arguments from the given buffer and calls the member function passed with the
 * extracted arguments.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] fn - member function to call
 * @param[in,out] obj - object to use
 * @remarks The given buffer may be modified for endian conversion.
 * @tparam R - function return value type
 * @tparam C - class type
 * @tparam Args - argument types
 */
template <typename R, typename C, typename ...Args>
static CallResult<R> callWith(FrameHandlerArgs & hal, R(C::* fn)(Args...), C & obj) {
	CallResult<R> res;
	tuple<Args...> args;
	res.success = extractArgs(hal, args);
	if ( res.success ) callUnpacked(res, fn, obj, args, typename ArgIndexGen<sizeof...(Args)>::type());
	return res;
}


/**
 * Extracts the arguments from the given buffer and calls the member function passed with the
 * extracted arguments.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] fn - member function to call
 * @param[in,out] obj - object to use
 * @remarks The given buffer may be modified for endian conversion.
 * @tparam R - function return value type
 * @tparam C - class type
 * @tparam Args - argument types
 */
template <typename R, typename C, typename ...Args>
static CallResult<R> callWith(FrameHandlerArgs & hal, R(C::* fn)(Args...) const, C & obj) {
	CallResult<R> res;
	tuple<Args...> args;
	res.success = extractArgs(hal, args);
	if ( res.success ) callUnpacked(res, fn, obj, args, typename ArgIndexGen<sizeof...(Args)>::type());
	return res;
}


#endif /* __UTILITY_HPP__ */
