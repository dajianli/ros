
#pragma once

#define EPS 0.00000001
#define DBLCMPEQ( d1, d2 ) ( fabs( (d1) - (d2) ) <= EPS )
#define DBLCMPLE( d1, d2 ) ( (d1) <= (d2) + EPS )
#define DBLCMPGE( d1, d2 ) ( (d1) >= (d2) - EPS )
#define DBLCMPLT( d1, d2 ) ( (d1) < (d2) - EPS )
#define DBLCMPGT( d1, d2 ) ( (d1) > (d2) + EPS )
#define ISBETWEEN(p,a,b) ( ( DBLCMPLT( (a), (b) ) ? ( DBLCMPGE( (p), (a) ) && DBLCMPLE( (p), (b) ) ) : \
    ( DBLCMPGE( (p), (b) ) && DBLCMPLE( (p), (a) ) ) ) ? true : false)

