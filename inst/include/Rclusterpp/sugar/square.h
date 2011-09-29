#ifndef RCLUSTERPP_SUGAR_SQUARE_H
#define RCLUSTERPP_SUGAR_SQUARE_H

namespace Rcpp {
	namespace sugar {

		template <bool NA, int RTYPE>
		class square__impl{
			public:
				typedef typename Rcpp::traits::storage_type<RTYPE>::type STORAGE;

				inline STORAGE get(STORAGE x) const {
					return Rcpp::traits::is_na<RTYPE>(x) ? NA_INTEGER : x * x;
				}
		};

		template <int RTYPE>
		class square__impl<false,RTYPE> {
			public:
				typedef typename Rcpp::traits::storage_type<RTYPE>::type STORAGE;
				
				inline STORAGE get( STORAGE x) const {
					return x * x;
				}
		};

	
		template <int RTYPE, bool NA, typename T>
		class Square : public Rcpp::VectorBase<REALSXP, NA, Square<RTYPE,NA,T> > {
			public:
				typedef typename Rcpp::VectorBase<RTYPE,NA,T> VEC_TYPE;
				typedef typename Rcpp::traits::storage_type<RTYPE>::type STORAGE;
				typedef square__impl<NA,RTYPE> OPERATOR;

				Square(const VEC_TYPE& object_) : object(object_), op() {}

				inline STORAGE operator[]( int i ) const {
					return op.get( object[i] );
				}
				inline int size() const { return object.size() ; }

			private:
				const VEC_TYPE& object;
				OPERATOR op;
		};
	
	} // sugar

	template <bool NA, typename T>
	inline sugar::Square<INTSXP,NA,T> square(const VectorBase<INTSXP,NA,T>& t) {
		return sugar::Square<INTSXP,NA,T>( t );
	}

	template <bool NA, typename T>
	inline sugar::Square<REALSXP,NA,T> square(const VectorBase<REALSXP,NA,T>& t) {
		return sugar::Square<REALSXP,NA,T>( t );
	}

} // Rcpp

#endif


