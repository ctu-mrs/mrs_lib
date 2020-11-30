#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include <mrs_lib/vector_converter.h>
#pragma GCC diagnostic pop

#include <impl/vector_converter_types.h>

// Boost preprocessor lib for cartesian product of type lists
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>

/* PREPROCESSOR BLACK MAGIC, NO TOUCHY TOUCHY! //{ */

#define INITIALIZE_TEMPLATES(FROM_TYPE, TO_TYPE) \
  template TO_TYPE mrs_lib::convert<TO_TYPE>(const FROM_TYPE&);

#define EVAL(...) __VA_ARGS__
#define INITIALIZE_TEMPLATES_SEMI_DELIM(R,SEQ_X) EVAL(INITIALIZE_TEMPLATES BOOST_PP_SEQ_TO_TUPLE(SEQ_X));
#define INITIALIZE_TEMPLATES_CARTESIAN(TUP_A,TUP_B) \
   BOOST_PP_SEQ_FOR_EACH_PRODUCT \
   ( INITIALIZE_TEMPLATES_SEMI_DELIM, \
     (BOOST_PP_TUPLE_TO_SEQ(TUP_A)) \
     (BOOST_PP_TUPLE_TO_SEQ(TUP_B)) \
   )

INITIALIZE_TEMPLATES_CARTESIAN((VC_TYPE_LIST), (VC_TYPE_LIST))

//}

