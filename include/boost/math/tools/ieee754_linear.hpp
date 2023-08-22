#ifndef BOOST_MATH_TOOLS_IEEE754_BITSPACE_HPP
#define BOOST_MATH_TOOLS_IEEE754_BITSPACE_HPP

#ifdef _MSC_VER
#pragma once
#endif

#include <boost/math/special_functions/next.hpp>

#include <type_traits>

namespace boost {
namespace math {
namespace tools {
namespace detail {
namespace bits_ieee754 {

   // The `IsFloat32` class contains a static constexpr method `value()` that
   // returns true if the type T is a 32 bit floating point type. This duck type
   // test for 32 bit float types returns true for the following types:
   //   - `float`
   //   - `_Float32` (NOTE: not a type alias for `float`)
   //   - `std_real_concept` when emulating a 32 bit float with EMULATE32
   //   - other types that seem to be 32 bit floats
   class IsFloat32 {
   public:
      template <typename T>
      static constexpr bool value() {
         return std::numeric_limits<T>::is_iec559 &&
                std::numeric_limits<T>::radix == 2 &&
                std::numeric_limits<T>::digits == 24 &&  // Mantissa has 23 bits + 1 implicit bit
                sizeof(T) == 4;
      }
   };

   // The `IsFloat64` class contains a static constexpr method `value()` that
   // returns true if the type T is a 64 bit floating point type. This duck type
   // test for 64 bit float types returns true for the following types:
   //   - `double`
   //   - `_Float64` (NOTE: not a type alias for `double`)
   //   - `std_real_concept` when emulating a 64 bit float with EMULATE64
   //   - other types that seem to be 64 bit floats
   class IsFloat64 {
   public:
      template <typename T>
      static constexpr bool value() {
         return std::numeric_limits<T>::is_iec559 &&
                std::numeric_limits<T>::radix == 2 &&
                std::numeric_limits<T>::digits == 53 &&  // Mantissa has 52 bits + 1 implicit bit
                sizeof(T) == 8;
      }
   };

   // The `IsFloat128` class contains a static constexpr method `value()` that
   // returns true if the type T is a 128 bit floating point type. This duck type
   // test for 128 bit float types returns true for the following types:
   //   - `__float128`
   //   - `_Float128` (NOTE: not a type alias for `__float128`)
   //   - `std_real_concept` when emulating a 128 bit float with EMULATE128
   //   - other types that seem to be 128 bit floats
   class IsFloat128 {
   public:
      template <typename T>
      static constexpr bool value() {
         return std::numeric_limits<T>::is_iec559 &&
                std::numeric_limits<T>::radix == 2 &&
                std::numeric_limits<T>::digits == 113 &&  // Mantissa has 112 bits + 1 implicit bit
                sizeof(T) == 16;
      }
   };

   // The `Layout_IEEE754Linear` class represents IEEE 754 floating point types
   // for which increasing float values result in increasing bit values, and for
   // which there is a corresponding unsigned integer type U that has the same
   // number of bits as T. Types that satisfy these conditions include 32, 64,
   // and 128 bit floats. These types can be used with efficient algorithms.
   //
   // NOTE: the 80 bit `long double` is not "linear" due to the "integer part". See:
   //       https://en.wikipedia.org/wiki/Extended_precision#x86_extended_precision_format
   template <typename T, typename U>
   class Layout_IEEE754Linear {
      static_assert(std::numeric_limits<T>::is_iec559, "Type must be IEEE 754 floating point.");
      static_assert(std::is_unsigned<U>::value, "U must be an unsigned integer type.");
      static_assert(sizeof(T) == sizeof(U), "Type and uint size must be the same.");
   };

   // The `Layout_NotSpecOrRadix2` class contains a static constexpr method
   // `value()` that returns true if the type T is not specialized or if the
   // radix of the type T is 2.
   class Layout_NotSpecOrRadix2{
   public:
      template <typename T>
      static constexpr bool value() {
         return !std::numeric_limits<T>::is_specialized || (std::numeric_limits<T>::radix == 2);
      }
   };

   // The `Layout_SpecAndRadixNot2` class represents floating point types
   // that are specialized and for which the radix is not equal to 2.
   template <typename T>
   class Layout_SpecAndRadixNot2 {};

   // The `get_layout<T>()` method of this class identifies the layout type
   // for floating point type T.
   class LayoutIdentifier {
   public:
      // Layout: 32 bit linear
      template <typename T>
      static typename std::enable_if<IsFloat32::value<T>(), Layout_IEEE754Linear<T, std::uint32_t>>::type
      get_layout() { return Layout_IEEE754Linear<T, std::uint32_t>(); }

      // Layout: 64 bit linear
      template <typename T>
      static typename std::enable_if<IsFloat64::value<T>(), Layout_IEEE754Linear<T, std::uint64_t>>::type
      get_layout() { return Layout_IEEE754Linear<T, std::uint64_t>(); }

      // Layout: 128 bit linear
#if defined(BOOST_HAS_INT128) && defined(BOOST_HAS_FLOAT128)
      // NOTE: returns Layout_IEEE754Linear<__float128, ...> instead of Layout_IEEE754Linear<T, ...>
      //       in order to work with boost::multiprecision::float128, which is a wrapper
      //       around __float128.
      template <typename T>
      static typename std::enable_if<IsFloat128::value<T>(), Layout_IEEE754Linear<__float128, boost::uint128_type>>::type
      get_layout() { return Layout_IEEE754Linear<__float128, boost::uint128_type>(); }

      template <typename T>
      static constexpr bool is_layout_nonlinear() {
         return !IsFloat32::value<T>() &&
                !IsFloat64::value<T>() &&
                !IsFloat128::value<T>();
      }
#else
      template <typename T>
      static constexpr bool is_layout_nonlinear() {
         return !IsFloat32::value<T>() &&
                !IsFloat64::value<T>();
      }
#endif

      // Layout: non-specialized or radix 2
      template <typename T>
      static typename std::enable_if<is_layout_nonlinear<T>() && Layout_NotSpecOrRadix2::value<T>(), Layout_NotSpecOrRadix2>::type
      get_layout() { return Layout_NotSpecOrRadix2(); }

      // Layout: specialized and radix not 2
      template <typename T>
      static typename std::enable_if<is_layout_nonlinear<T>() && !Layout_NotSpecOrRadix2::value<T>(), Layout_SpecAndRadixNot2<T>>::type
      get_layout() { return Layout_SpecAndRadixNot2<T>(); }
   };

   // Predeclare BitsDeflated
   template <typename T, typename U>
   class BitsDeflated;

   // Predeclare BitsInflated
   template <typename T, typename U>
   class BitsInflated;

   //
   // Consider the bitspace representation of a floating point type `T` above
   // for the case where denorm support is turned on. The bitspace is linear.
   // Increasing values in bitspace correspond to increasing values in float space
   // from `-Inf` to `+Inf`. This is illustrated below.
   //   
   //   Smallest                                          Largest
   // BitsInflated                                      BitsInflated
   //    Value                                             Value
   //      |--------------------|---|---|--------------------|
   //    -Inf                 -min  0  min                 +Inf
   //
   // Now consider the case where denorm support is turned off. The result is
   // gaps in bitspace around `0`. This is illustrated below where the `***` marks
   // the locations of denormal numbers.
   //
   //      |--------------------|***|***|--------------------|
   //    -Inf                 -min  0  min                 +Inf
   //
   // Doing operations in inflated bitspace can give the wrong answer if one
   // passes into the denormal numbers. But doing operations is still possible
   // because the bitspace is still linear, if one removes the denormal numbers
   // as shown below.
   //
   //         |--------------------|||--------------------|
   //       -Inf                    0                   +Inf
   // 
   // The `Denormflator` converts between inflated and deflated bitspace representations
   // to compensate for gaps in the bitspace caused by denormal numbers. The `deflate`
   // operation shifts the bitspace representation toward `0`. The `inflate` operation
   // shifts the bitspace representation away from `0`. These operations only have 
   // an effect if denorm support is turned off. If denorm support is turned on, then
   // the shift is zero.
   //
   template <typename T, typename U>
   class Denormflator : public Layout_IEEE754Linear<T, U> {
   public:
      Denormflator() : has_denorm_(calc_has_denorm()) {}

      BitsDeflated<T, U> deflate(const BitsInflated<T, U>& bit_view) const {
         const U penalty = bits_denorm_shift();
         const U mag_un = bit_view.mag();
         const U mag_sh = (has_denorm_ | (mag_un < penalty)) ? mag_un : mag_un - penalty;
         return BitsDeflated<T, U>(bit_view.sign_bit(), mag_sh);
      }

      BitsInflated<T,U> inflate(const BitsDeflated<T,U>& b) const {
         const U mag = b.mag();
         const U mag_inflated = (has_denorm_ | (mag == 0)) ? mag : mag + bits_denorm_shift();
         return BitsInflated<T, U>(b.sign_bit(), mag_inflated);
      }

   private:
      // Denorm penalty
      static constexpr U bits_denorm_shift() { return (U(1) << (std::numeric_limits<T>::digits - 1)) - 1; }
      
      static bool calc_has_denorm() {
         return boost::math::detail::get_smallest_value<T>() != (std::numeric_limits<T>::min)();
      }

      bool has_denorm_;
   };

   // Base class for the `BitsInflated` and `BitsDeflated` classes.
   template <typename T, typename U>
   class BitsFromZero: public Layout_IEEE754Linear<T, U> {
   public:
      bool sign_bit() const { return sign_bit_; }
      const U& mag() const { return mag_; }
      U& mag() { return mag_; }

   protected:
      BitsFromZero(const bool sign, const U mag) : sign_bit_(sign), mag_(mag) {}

      BitsFromZero(const T x) { 
         U bits_;
         std::memcpy(&bits_, &x, sizeof(U));
         sign_bit_ = bits_ & BitsInflated<T,U>::bits_sign_mask();
         mag_ = bits_ & ~BitsInflated<T,U>::bits_sign_mask();
      }

      void flip_sign_bit() { sign_bit_ = !sign_bit_; }

   private:
      bool sign_bit_;
      U mag_;
   };

   // The inflated bitspace representation of a floating point type `T`.
   template <typename T, typename U>
   class BitsInflated : public BitsFromZero<T, U> {
   public:
      BitsInflated(const T x) : BitsFromZero<T, U>(x) {}
      BitsInflated(const bool sign, const U mag) : BitsFromZero<T, U>(sign, mag) {}
      
      // The sign bit is 1, all other bits are 0
      static constexpr U bits_sign_mask() { return U(1) << (sizeof(U) * 8 - 1); }

      T reinterpret_as_float() const {
         T f_out;
         std::memcpy(&f_out, &this->mag(), sizeof(T));
         return this->sign_bit() ? -f_out : f_out;
      }

      void divide_by_2() { this->mag() >>= 1; }
   };

   // The deflated bitspace representation of a floating point type `T`. 
   template <typename T, typename U>
   class BitsDeflated : public BitsFromZero<T, U> {
   public:
      BitsDeflated(const bool sign, const U mag) : BitsFromZero<T, U>(sign, mag) {}

      T static_cast_int_value_to_float() const {
         T mag_float = static_cast<T>(this->mag());
         return this->sign_bit() ? -mag_float : mag_float;
      }

      // Move over `n` in bitspace
      BitsDeflated<T,U> operator+(int n) const {
         return BitsDeflated<T,U>(this->sign_bit(), this->mag() + n);
      }

      BitsDeflated<T,U> operator-(const BitsDeflated<T,U>& y) const {
         auto y_copy = y;
         y_copy.flip_sign_bit();
         return *this + y_copy;
      }

      BitsDeflated<T,U> operator+(const BitsDeflated<T,U>& y) const {
         // Gaurantee that y has the larger magnitude
         if (y.mag() < this->mag()) { return y + *this; }

         // Call *this x
         const BitsDeflated<T, U>& x = *this;

         const U mag_x = x.mag();
         const U mag_y = y.mag();

         // Calculate the deflated sum in bits (always positive)
         U bits_sum = (x.sign_bit() == y.sign_bit()) ? (mag_y + mag_x) 
                                                     : (mag_y - mag_x);

         // Sum always has the sign of the bigger magnitude (y)
         return BitsDeflated<T,U>(y.sign_bit(), bits_sum);
      }

      BitsDeflated<T,U>& operator>>(const int n) {
         this->mag() >>= n;
         return *this;
      }
   };

   //
   // Static dispatch
   //
   class calc {
   public:
      // BitDist //
      template <typename T>
      static T bit_dist(T x, T y) { return bit_dist(x, y, LayoutIdentifier::get_layout<T>()); }

      // Midpoint //
      template <typename T>
      static T midpoint(T x, T y) { return midpoint(x, y, LayoutIdentifier::get_layout<T>()); }

      // FloatAdvance //
      template <typename T>
      static T float_advance(T x, int n) { return float_advance(x, n, LayoutIdentifier::get_layout<T>()); }

      // FloatNext //
      template <typename T>
      static T float_next(T x) { return float_next(x, LayoutIdentifier::get_layout<T>()); }

      // FloatPrior //
      template <typename T>
      static T float_prior(T x) { return float_prior(x, LayoutIdentifier::get_layout<T>()); }

   private:
      // BitDist //
      template <typename T, typename U>
      static T bit_dist(T x, T y, Layout_IEEE754Linear<T, U> b) {
         const auto df = Denormflator<T, U>();
         const auto y_def = df.deflate(BitsInflated<T, U>(y));
         const auto x_def = df.deflate(BitsInflated<T, U>(x));
         const auto y_def_minus_x_def = y_def - x_def;
         return y_def_minus_x_def.static_cast_int_value_to_float();
      }
      template <typename T>
      static T bit_dist(T x, T y, Layout_NotSpecOrRadix2) {
         return boost::math::float_distance(x, y);
      }

      // Midpoint //
      template <typename T, typename U>
      static T midpoint(T x, T y, Layout_IEEE754Linear<T, U> b) {
         const auto df = Denormflator<T, U>();
         const auto y_def = df.deflate(BitsInflated<T, U>(y));
         const auto x_def = df.deflate(BitsInflated<T, U>(x));
         const auto xy_def_midpoint = (y_def + x_def) >> 1;  // Add then divide by 2
         const auto xy_midpoint = df.inflate(xy_def_midpoint);
         return xy_midpoint.reinterpret_as_float();
      }
      template <typename T>
      static T midpoint(T x, T y, Layout_NotSpecOrRadix2) {
         // TODO: fallback method goes here.
         static_assert(sizeof(T) == 0);
         return T();
      }

      // FloatAdvance // 
      template <typename T, typename U>
      static T float_advance(T x, int n, Layout_IEEE754Linear<T, U> b) {
         const auto df = Denormflator<T, U>();
         const auto x_def = df.deflate(BitsInflated<T, U>(x));
         const auto x_def_plus_n = x_def + n;
         const auto x_plus_n = df.inflate(x_def_plus_n);
         return x_plus_n.reinterpret_as_float();
      }
      template <typename T>
      static T float_advance(T x, int n, Layout_NotSpecOrRadix2) { return boost::math::float_advance(x, n); }

      // FloatNext //
      template <typename T, typename U>
      static T float_next(T x, Layout_IEEE754Linear<T, U> b) { return float_advance(x, 1, b); }
      template <typename T>
      static T float_next(T x, Layout_NotSpecOrRadix2) { return boost::math::float_next(x); }
      
      // FloatPrior //
      template <typename T, typename U>
      static T float_prior(T x, Layout_IEEE754Linear<T, U> b) { return float_advance(x, -1, b); }
      template <typename T>
      static T float_prior(T x, Layout_NotSpecOrRadix2) { return boost::math::float_prior(x); }
   };

}  // namespace bits_ieee754
}  // namespace detail
}  // namespace tools
}  // namespace math
}  // namespace boost   

#endif  // BOOST_MATH_TOOLS_IEEE754_BITSPACE_HPP