// boost\math\tools\promotion.hpp

// Copyright John Maddock 2006.
// Copyright Paul A. Bristow 2006.
// Copyright Matt Borland 2023.

// Use, modification and distribution are subject to the
// Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt)

// Promote arguments functions to allow math functions to have arguments
// provided as integer OR real (floating-point, built-in or UDT)
// (called ArithmeticType in functions that use promotion)
// that help to reduce the risk of creating multiple instantiations.
// Allows creation of an inline wrapper that forwards to a foo(RT, RT) function,
// so you never get to instantiate any mixed foo(RT, IT) functions.

#ifndef BOOST_MATH_PROMOTION_HPP
#define BOOST_MATH_PROMOTION_HPP

#ifdef _MSC_VER
#pragma once
#endif

#include <boost/math/tools/config.hpp>
#include <type_traits>

#if __has_include(<stdfloat>)
#  include <stdfloat>
#endif

namespace boost
{
  namespace math
  {
    namespace tools
    {
      // If either T1 or T2 is an integer type,
      // pretend it was a double (for the purposes of further analysis).
      // Then pick the wider of the two floating-point types
      // as the actual signature to forward to.
      // For example:
      // foo(int, short) -> double foo(double, double);
      // foo(int, float) -> double foo(double, double);
      // Note: NOT float foo(float, float)
      // foo(int, double) -> foo(double, double);
      // foo(double, float) -> double foo(double, double);
      // foo(double, float) -> double foo(double, double);
      // foo(any-int-or-float-type, long double) -> foo(long double, long double);
      // but ONLY float foo(float, float) is unchanged.
      // So the only way to get an entirely float version is to call foo(1.F, 2.F),
      // But since most (all?) the math functions convert to double internally,
      // probably there would not be the hoped-for gain by using float here.



      
      ////// Promotion layers / Tools
      //
      // Tool: `promote_integral_to_double<T>`:
      //   - Promotes `T` to `double` if `T` is an integer type as identified by
      //     `std::is_integral`, otherwise is `T`
      //
      // One Argument Promotion Rules:
      //   - Applies `promote_integral_to_double<T>` to `T`
      //
      // Two Argument Promotion Rules:
      //   - Applies the one argument promotion rules to `T1` to get `TP1`
      //   - Applies the one argument promotion rules to `T2` to get `TP2`
      //   - If `TP1` and `TP2` are both floating point types, as identified by
      //     `std::is_floating_point`, then return the type with more bits
      //   - Otherwise return the type that both types can be converted into
      //




      // This follows the C-compatible conversion rules of pow, etc
      // where pow(int, float) is converted to pow(double, double).

      template <class T, bool = std::is_integral<T>::value>
      struct promote_integral_to_double {
         using type = double;
      };

      template <class T>
      struct promote_integral_to_double<T, false> {
         using type = T;
      };

      template <class T>
      struct promote_arg {
         using type = typename promote_integral_to_double<T>::type;
      };

      #if defined(__STDCPP_FLOAT64_T__) && LDBL_MANT_DIG == 53 && LDBL_MAX_EXP == 1024
      template <> struct promote_arg<long double> { using type = std::float64_t; };
      #endif

      ////////////////////////////////////////////////////////////////////////////////////////
      

      template <class T1, class T2>
      struct promote_args_2_equal {
         static_assert(std::is_floating_point<T1>::value, "T1 must be a floating point type");
         static_assert(std::is_floating_point<T2>::value, "T2 must be a floating point type");
         static_assert(sizeof(T1) != sizeof(T2), "Missing promotion specialization for equal sized types");
      };

      ////////////////////////////////////////////////////////////////////////////////////////

      template <class T1, class T2, bool = (sizeof(T1) > sizeof(T2)), bool = (sizeof(T1) < sizeof(T2))>
      struct promote_args_2_ff {
         using type = typename promote_args_2_equal<T1, T2>::type;
      };

      template <class T>
      struct promote_args_2_ff<T, T, false, false> {
         using type = T;
      };

      template <class T1, class T2>
      struct promote_args_2_ff<T1, T2, true, false> {
         using type = T1;
      };

      template <class T1, class T2>
      struct promote_args_2_ff<T1, T2, false, true> {
         using type = T2;
      };

      ////////////////////////////////////////////////////////////////////////////////////////

      template <class T1, class T2, bool = std::is_floating_point<T1>::value, bool = std::is_floating_point<T2>::value>
      struct pa2_no_integral {
         using type = typename std::conditional<
            !std::is_floating_point<T2>::value && std::is_convertible<T1, T2>::value, 
            T2,
            T1>::type;
      };

      template <class T1, class T2>
      struct pa2_no_integral<T1, T2, true, true> {
         using type = typename promote_args_2_ff<T1, T2>::type;
      };

      ////////////////////////////////////////////////////////////////////////////////////////


#if 0
      template <class T1, class T2>
      struct promote_args_2 {
         // Promote both parameters & pick the wider of the two floating-point types.
         using T1P = typename promote_arg<T1>::type; // T1 perhaps promoted.
         using T2P = typename promote_arg<T2>::type; // T2 perhaps promoted.
         using type = typename pa2_no_integral<T1P, T2P>::type;
      };
#else
      // // Predeclare promote_args_2
      // template <class T>
      // struct promote_with_double {
      //    using TP = typename promote_arg<T>::type; // T2 perhaps promoted.
      //    using type = typename pa2_no_integral<double, TP>::type;
      // };
      
      template <class T1, class T2>
      struct promote_args_2 {
         using type = typename pa2_no_integral<
            typename promote_arg<T1>::type,
            typename promote_arg<T2>::type
         >::type;

         // static constexpr bool is_int_1 = std::is_integral<T1>::value;
         // static constexpr bool is_int_2 = std::is_integral<T2>::value;
         
         // using type =
         // typename std::conditional_t<!is_int_1 && !is_int_2, 
         //    typename pa2_no_integral<T1, T2>::type,
         //    typename pa2_no_integral<
         //       typename std::conditional_t<is_int_1, double, typename promote_arg<T1>::type>::type,
         //       typename std::conditional_t<is_int_2, double, typename promote_arg<T2>::type>::type
         //    >::type
         // >;
         
         // std::is_integral<T1>::value, 
         //    typename std::conditional_t<std::is_integral<T2>::value, 

         //    typename pa2_no_integral<double, typename promote_arg<T2>::type>::type,
         
         
         //       typename pa2_no_integral<typename promote_arg<T1>::type, double>::type,
         //       typename pa2_no_integral<T1, T2>::type
         //    >
         // >;



         // using type =
         // typename std::conditional_t<std::is_integral<T1>::value,
         //    // typename promote_with_double<T2>::type,
         //    typename pa2_no_integral<double, typename promote_arg<T2>::type>::type,
         //    typename std::conditional_t<std::is_integral<T2>::value,
         //       // typename promote_with_double<T1>::type,
         //       typename pa2_no_integral<typename promote_arg<T1>::type, double>::type,
         //       typename pa2_no_integral<T1, T2>::type
         //    >
         // >;
      };
#endif 

      template <> struct promote_args_2<void, void> { using type = void; };

      template <class T> struct promote_args_2<T, void> {
         using type = typename promote_arg<T>::type;  // T perhaps promoted
      };

      template <typename T, typename U>
      using promote_args_2_t = typename promote_args_2<T, U>::type;

      ////////////////////////////////////////////////////////////////////////////////////////

      template <class T1, class T2=void, class T3=void, class T4=void, class T5=void, class T6=void>
      struct promote_args_permissive {
         using type = 
         typename promote_args_2<typename std::remove_cv<T1>::type, 
            typename promote_args_2<typename std::remove_cv<T2>::type,
               typename promote_args_2<typename std::remove_cv<T3>::type,
                  typename promote_args_2<typename std::remove_cv<T4>::type,
                     typename promote_args_2<typename std::remove_cv<T5>::type,
                                             typename std::remove_cv<T6>::type
                     >::type
                  >::type
               >::type
            >::type
         >::type;
      };

      template <class T1, class T2=void, class T3=void, class T4=void, class T5=void, class T6=void>
      using promote_args_permissive_t = typename promote_args_permissive<T1, T2, T3, T4, T5, T6>::type;

      template <class T1, class T2=void, class T3=void, class T4=void, class T5=void, class T6=void>
      struct promote_args {
         using type = promote_args_permissive_t<T1, T2, T3, T4, T5, T6>;

#if defined(BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)
         //
         // Guard against use of long double if it's not supported:
         //
         static_assert((0 == std::is_same<type, long double>::value), "Sorry, but this platform does not have sufficient long double support for the special functions to be reliably implemented.");
#endif
      };

      template <class T1, class T2=void, class T3=void, class T4=void, class T5=void, class T6=void>
      using promote_args_t = typename promote_args<T1, T2, T3, T4, T5, T6>::type;

    } // namespace tools
  } // namespace math
} // namespace boost

#endif // BOOST_MATH_PROMOTION_HPP

