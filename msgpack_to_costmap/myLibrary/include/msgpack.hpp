/*
----------------------------------------------------------------------
MSGPACKALT :: a simple binary serialisation library
http://code.google.com/p/msgpackalt
----------------------------------------------------------------------
*/
/** \file msgpackalt.hpp
 *  \brief A simple C++ interface, using overloaded operators (<< and >>)
 *  to simplify code syntax, with support for STL strings, maps, vectors
 *  and dictionaries.
*/
#ifndef MSGPACK_HPP
#define MSGPACK_HPP

#ifdef MSGPACK_QT				/* enable the Qt interface? */
#include <QByteArray>
#include <QHash>
#include <QVector>
#else
#ifndef MSGPACK_STL
#define MSGPACK_STL	            /* by default use STL */
#endif
#endif

#ifdef MSGPACK_STL			/* enable the STL interface? */
#include <string>
#include <map>
#include <vector>
#include <set>
#include <cstdio>	  	    /* snprintf */
#include <cstring>
#endif
#include <stdexcept>

#ifdef _MSC_VER			     /* visual c++ fixes */
#define snprintf _snprintf
#pragma warning (disable: 4996)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

#include "msgpack-functions.hpp"

namespace msgpack
{
/// Takes the msgpackalt return code and throws a relevant exception on error
/** The underlying msgpack C library returns negative values denoting failure.
 *	A different exception is thrown for each error type so errors can be trapped.
 *
 *	Note that all can be caught as std::exception and the what() function can
 *	be used to interpret the cause.
 *
 *	MSGPACK_TYPEERR: function call does not match the expected type code
 *	                    throws std::out_of_range
 *	MSGPACK_MEMERR:  failed to allocate sufficient memory for the operation
 *	                    throws std::runtime_error
 *	MSGPACK_ARGERR:  function called with invalid argument
 *						throws std::invalid_argument
 *	other error:     received negative return code, but unknown cause
 *						throws std::exception
 */
INLINE void msgpack_assert( MSGPACK_ERR code, const char* f )
{
    if ( code < MSGPACK_SUCCESS ) {	// error codes are negative
        char buffer[128];
        if ( code == MSGPACK_TYPEERR ) {
            snprintf( buffer, 128, "Unexpected type code in %s", f );
            throw std::out_of_range(buffer);
        } else if ( code == MSGPACK_MEMERR ) {
            snprintf( buffer, 128, "Memory allocation/access error in %s", f );
            throw std::runtime_error(buffer);
        } else if ( code == MSGPACK_ARGERR ) {
            snprintf( buffer, 128, "Invalid argument passed inside %s", f );
            throw std::invalid_argument(buffer);
        } else {
            snprintf( buffer, 128, "Unknown error code %i during %s", code, f );
            throw std::range_error(buffer);
        }
    }
}
#define MSGPACK_ASSERT(x) msgpack_assert(x,__PRETTY_FUNCTION__)

/// The serialisation class which packs data in the MessagePack format
class Packer
{
public:
    /// default copy constructor; allocate a msgpack packer object
    Packer( )
    {
        this->m = msgpack_pack_init( );
    }
    /// default destructor; cleans up any allocated memory
    ~Packer( )
    {
        msgpack_pack_free( this->m );
        this->m = NULL;
    }

    /// return pointer to underlying C structs -- internal use only
    msgpack_p* ptr( )
    {
        return this->m;
    }

    /// return the number of bytes packed so far
    uint32_t len( ) const
    {
        return msgpack_get_len( this->m );
    }
    /// return a pointer to a copy of the data
    void* duplicate( uint32_t &n ) const
    {
        n = len();
        if ( n == 0 ) return NULL;
        void *x = malloc( n );
        memcpy( x, m->buffer, n );
        return x;
    }
    /// clears the contents of the internal buffer
    void clear( )
    {
        if ( this->m ) m->p = m->buffer;
    }

#ifdef MSGPACK_STL
    /// return an STL string with the contents of the buffer
    std::string string( ) const
    {
        const byte* b = NULL;
        uint32_t n = 0;
        MSGPACK_ASSERT( msgpack_get_buffer( this->m, &b, &n ));
        return std::string(( char* )b,n );
    }
#endif
#ifdef MSGPACK_QT
    QByteArray string( ) const
    {
        const byte* b = NULL;
        uint32_t n = 0;
        MSGPACK_ASSERT( msgpack_get_buffer( this->m, &b, &n ));
        return QByteArray(( char* )b,n );
    }
#endif

    /// LOW-LEVEL: specifies an array is to follow, with elements consisting of the next "n" packing calls.
    void start_array( uint32_t n )
    {
        MSGPACK_ASSERT( msgpack_pack_array( this->m, n ));
    }
    /// LOW-LEVEL: specifies a map is to follow, with "n" sets of keys and values consisting of the next 2*n calls.
    void start_map( uint32_t n )
    {
        MSGPACK_ASSERT( msgpack_pack_map( this->m, n ));
    }

    // *********************************** PACKING FUNCTIONS ***********************************
    /// Pack a boolean value
    Packer& operator<<( bool b )
    {
        MSGPACK_ASSERT( msgpack_pack_bool( this->m, b ));
        return *this;
    }
    /// Pack an 8-bit unsigned int
    Packer& operator<<( const uint8_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_uint8( this->m, x ));
        return *this;
    }
    /// Pack a 16-bit unsigned int
    Packer& operator<<( const uint16_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_uint16( this->m, x ));
        return *this;
    }
    /// Pack a 32-bit unsigned int
    Packer& operator<<( const uint32_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_uint32( this->m, x ));
        return *this;
    }
    /// Pack a 64-bit unsigned int
    Packer& operator<<( const uint64_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_uint64( this->m, x ));
        return *this;
    }
	/// Pack a pointer
	template<typename T>
    Packer& operator<<( const T* p )
    {
		uint64_t value = (uint64_t) p;
        MSGPACK_ASSERT( msgpack_pack_uint64( this->m, value ) );
        return *this;
    }


    /// Pack an 8-bit signed int
    Packer& operator<<( const int8_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_int8( this->m, x ));
        return *this;
    }
    /// Pack a 16-bit signed int
    Packer& operator<<( const int16_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_int16( this->m, x ));
        return *this;
    }
    /// Pack a 32-bit signed int
    Packer& operator<<( const int32_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_int32( this->m, x ));
        return *this;
    }
    /// Pack a 64-bit signed int
    Packer& operator<<( const int64_t &x )
    {
        MSGPACK_ASSERT( msgpack_pack_int64( this->m, x ));
        return *this;
    }

    /// Pack a 32-bit float
    Packer& operator<<( const float &x )
    {
        MSGPACK_ASSERT( msgpack_pack_float( this->m, x ));
        return *this;
    }
    /// Pack a double (64-bit float)
    Packer& operator<<( const double &x )
    {
        MSGPACK_ASSERT( msgpack_pack_double( this->m, x ));
        return *this;
    }

    /// Pack a C-style string as raw data
    Packer& operator<<( const char *s )
    {
        MSGPACK_ASSERT( msgpack_pack_str( this->m, s ));
        return *this;
    }
    /// Pack "n" bytes of raw data specified by the given pointer
    Packer& pack_raw( const void* data, const uint32_t n )
    {
        MSGPACK_ASSERT( msgpack_pack_raw( this->m, ( const byte* )data, n ));
        return *this;
    }
    /// Pack the "null" object
    Packer& pack_null( )
    {
        MSGPACK_ASSERT( msgpack_pack_null( this->m ));
        return *this;
    }
    /// Pack a C-style array of "n" points starting at the pointer "v"
    template<class T> Packer& pack_array( const T* v, const uint32_t n )
    {
        this->start_array( n );
        for ( uint32_t i = 0; i < n; ++i ) *this << v[i];
        return *this;
    }

    uint32_t append( const void* ptr, uint32_t n )
    {
        MSGPACK_ASSERT( msgpack_pack_append( this->m, ptr, n ));
        return len();
    }
    /// Append the contents of another Packer object
    Packer& operator<<( const Packer &p )
    {
        const byte* b = NULL;
        uint32_t n = 0;
        msgpack_get_buffer( p.m, &b, &n );
        append( b, n );
        return *this;
    }

#ifdef MSGPACK_STL
    /// Pack a std::string as raw data
    Packer& operator<<( const std::string &s )
    {
        this->pack_raw( s.data( ), s.size( ));
        return *this;
    }
    /// Pack an STL vector of any valid type
    template<class T> Packer& operator<<( const std::vector<T> &v )
    {
        return this->pack_array( &v[0], v.size( ));
    }
    /// Pack an STL set of any valid type
    template<class T> Packer& operator<<( const std::set<T> & s )
    {
        std::vector<T> v;
        for ( T e : s ){
            v.push_back( e );
        }
        return this->pack_array( &v[0], v.size( ));
    }
    /// Pack an STL map between any valid types
    template<class T, class U> Packer& operator<<( const std::map<T,U>
            &v )
    {
        this->start_map( v.size());
        for ( typename std::map<T,U>::const_iterator i = v.begin();
                i != v.end(); ++i ) *this << i->first << i->second;
        return *this;
    }
    /// Pack an STL pair of any valid types. (added by zb)
    template<typename T1, typename T2>
    Packer & operator << (const std::pair<T1,T2> & p )
    {
        *this << p.first << p.second;
        return *this;
    }

#endif
#ifdef MSGPACK_QT
    /// Pack a std::string as raw data
    Packer& operator<<( const QByteArray &s )
    {
        this->pack_raw( s.data(), s.size());
        return *this;
    }
    /// Pack an STL map between any valid types
    template<class T, class U> Packer& operator<<( const QHash<T,U> &v )
    {
        this->start_map( v.size());
        QHashIterator<T,U> i(v);
        while ( i.hasNext( )) {
            i.next();
            *this << i.key() << i.value();
        }
        return *this;
    }
    /// Pack an STL map between any valid types
    template<class T, class U> Packer& operator<<( const QMap<T,U> &v )
    {
        this->start_map( v.size());
        QMapIterator<T,U> i(v);
        while ( i.hasNext( )) {
            i.next();
            *this << i.key() << i.value();
        }
        return *this;
    }
#endif

protected:
    /// Underlying C Packer object
    msgpack_p *m;
    friend class Unpacker;

private:
    /// Pointers are not reference counted, so prevent automatic copies. Use the << operator to append instead.
    Packer& operator=( const Packer& P );
};

/// The deserialisation class which retrieves data packed in the MessagePack format from a binary string
class Unpacker
{
public:
    /// Default constructor: start with the empty string
    Unpacker( )
    {
        this->u = msgpack_unpack_init( NULL, 0, true );
    }
    /// Create Unpacker from a raw block of memory
    Unpacker( const byte *buffer, uint32_t len, bool copy = true )
    {
        this->u = msgpack_unpack_init( buffer, len, copy );
    }
#ifdef MSGPACK_STL
    Unpacker( const std::string &str )
    {
        this->u = msgpack_unpack_init(( const byte* )str.data(), str.size(),
                                      true );
    }
#endif
#ifdef MSGPACK_QT
    Unpacker( const QByteArray &data )
    {
        this->u = msgpack_unpack_init(( const byte* )data.data(), data.size(),
                                      true );
    }
#endif
    /// Default destructor
    ~Unpacker( )
    {
        if ( this->u ) msgpack_unpack_free( this->u );
        this->u = NULL;
    }

    /// Return pointer to underlying struct -- internal use only
    const byte* ptr( )
    {
        return this->u->p;
    }

    /// Return number of bytes remaining in the buffer to unpack
    uint32_t len( ) const
    {
        return msgpack_unpack_len( this->u );
    }
    /// Return the code of the next item to unpack
    int peek( ) const
    {
        return msgpack_unpack_peek( this->u );
    }
    /// Skip the next item in the buffer and return the number of bytes skipped
    int skip( )
    {
        return msgpack_unpack_skip( this->u );
    }
    /// Move the Unpacker back to the start of the buffer
    void restart( )
    {
        msgpack_unpack_setpos( this->u, 0 );
    }

    /// Clear the buffer
    void clear( )
    {
        this->u->end = this->u->p = this->u->end - this->u->max;
        this->u->max = 0;
    }
    /// Append data to the end of the buffer, e.g. streaming data, and return the total size of the buffer (not necessarily bytes remaining to be unpacked)
    uint32_t append( const byte *data, uint32_t len )
    {
        MSGPACK_ASSERT( msgpack_unpack_append( this->u, data, len ));
        return this->u->max;
    }
    /// Throw away the current buffer and copy the given data
    void set( const byte *data, uint32_t len )
    {
        this->clear( );
        this->append( data, len );
    }
#ifdef MSGPACK_STL
    /// Throw away the current buffer and copy the given string
    Unpacker& operator=( const std::string &s )
    {
        this->set(( const byte* )s.data( ), s.size( ));
        return *this;
    }
#endif
#ifdef MSGPACK_QT
    /// Throw away the current buffer and copy the given string
    Unpacker& operator=( const QByteArray &x )
    {
        this->set(( const byte* )x.data( ), x.size( ));
        return *this;
    }
#endif

    // *********************************** UNPACKING FUNCTIONS ***********************************
    /// Unpack a boolean value
    Unpacker& operator>>( bool &b )
    {
        int x = msgpack_unpack_bool( this->u );
        b = x > 0;
        MSGPACK_ASSERT(( MSGPACK_ERR )x );
        return *this;
    }

    /// Unpack an 8-bit unsigned int
    Unpacker& operator>>( uint8_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_uint8( this->u, &x ));
        return *this;
    }
    /// Unpack a 16-bit unsigned int
    Unpacker& operator>>( uint16_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_uint16( this->u, &x ));
        return *this;
    }
    /// Unpack a 32-bit unsigned int
    Unpacker& operator>>( uint32_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_uint32( this->u, &x ));
        return *this;
    }
    /// Unpack a 64-bit unsigned int
    Unpacker& operator>>( uint64_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_uint64( this->u, &x ));
        return *this;
    }
	/// Unpack a pointer
	template<typename T>
    Unpacker& operator>>( T* & x )
    {
		uint64_t value;
        MSGPACK_ASSERT( msgpack_unpack_uint64( this->u, &value ));
		x = (T*) value;
        return *this;
    }

    /// Unpack an 8-bit signed int
    Unpacker& operator>>( int8_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_int8( this->u, &x ));
        return *this;
    }
    /// Unpack a 16-bit signed int
    Unpacker& operator>>( int16_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_int16( this->u, &x ));
        return *this;
    }
    /// Unpack a 32-bit signed int
    Unpacker& operator>>( int32_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_int32( this->u, &x ));
        return *this;
    }
    /// Unpack a 64-bit signed int
    Unpacker& operator>>( int64_t &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_int64( this->u, &x ));
        return *this;
    }

    /// Unpack a U8 value
    Unpacker& operator>>( float &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_float( this->u, &x ));
        return *this;
    }
    /// Unpack a U8 value
    Unpacker& operator>>( double &x )
    {
        MSGPACK_ASSERT( msgpack_unpack_double( this->u, &x ));
        return *this;
    }

#ifdef MSGPACK_STL
    /// Unpack raw data into a std::string
    Unpacker& operator>>( std::string &s )
    {
        uint32_t n = 0;
        const void* b = unpack_raw( n );
        s = std::string(( const char* )b, n );
        return *this;
    }
    /// Unpack a vector of homogeneous (single typed) data into the given STL vector
    template<class T> Unpacker& operator>>( std::vector<T> &v )
    {
        uint32_t n = start_array( );
        T x;
        v.clear( );
        for ( uint32_t i = 0; i < n; ++i ) {
            *this >> x;
            v.push_back( x );
        }
        return *this;
    }
    /// Unpack a set of homogeneous (single typed) data into the given STL set
    template<class T> Unpacker& operator>>( std::set<T> & s )
    {
        uint32_t n = start_array( );
        T x;
        s.clear( );
        for ( uint32_t i = 0; i < n; ++i ) {
            *this >> x;
            s.insert( x );
        }
        return *this;
    }
    /// Unpack a map object with key and value types given by the STL map
    template<class T, class U> Unpacker& operator>>( std::map<T,U> &v )
    {
        uint32_t n = start_map( );
        T x;
        U y;
        v.clear( );
        for ( uint32_t i = 0; i < n; ++i ) {
            *this >> x >> y;
            v.insert( std::pair<T,U>( x,y ));
        }
        return *this;
    }
    /// Unpack a STL pair (added by zb)
    template<typename T1, typename T2>
    Unpacker & operator >> (std::pair<T1,T2> & p )
    {
        *this >> p.first >> p.second;
        return *this;
    }
#endif

#ifdef MSGPACK_QT
    /// Unpack raw data into a std::string
    Unpacker& operator>>( QByteArray &x )
    {
        uint32_t n = 0;
        const void* b = unpack_raw( n );
        x = QByteArray(( const char* )b, n );
        return *this;
    }
    /// Unpack a hash object
    template<class T, class U> Unpacker& operator>>( QHash<T,U> &v )
    {
        uint32_t n = start_map( );
        T x;
        U y;
        v.clear( );
        for ( uint32_t i = n; i > 0; --i ) {
            *this >> x >> y;
            v.insert( x,y );
        }
        return *this;
    }
    /// Unpack a map object
    template<class T, class U> Unpacker& operator>>( QMap<T,U> &v )
    {
        uint32_t n = start_map( );
        T x;
        U y;
        v.clear( );
        for ( uint32_t i = n; i > 0; --i ) {
            *this >> x >> y;
            v.insert( x,y );
        }
        return *this;
    }
#endif

    /// LOW-LEVEL: Expect the next object to be the start of an array. Return the number of entries N, comprising the next N unpack calls.
    uint32_t start_array( )
    {
        uint32_t n;
        MSGPACK_ASSERT( msgpack_unpack_array( this->u, &n ));
        return n;
    }
    /// LOW-LEVEL: Expect the next object to be the start of a map. Return the number of (key,value) pairs N, comprising the next 2*N unpack calls.
    uint32_t start_map( )
    {
        uint32_t n;
        MSGPACK_ASSERT( msgpack_unpack_map( this->u, &n ));
        return n;
    }

    const void* unpack_raw( uint32_t &n )
    {
        const byte* b;
        MSGPACK_ASSERT( msgpack_unpack_raw( this->u, &b, &n ));
        return b;
    }

protected:
    /// Underlying C Unpacker object
    msgpack_u *u;

private:
    /// Pointers are not reference counted, so prevent automatic copies. Use the << operator to append instead.
    Unpacker& operator=( const Unpacker& P );
};

/// A simple class containing a single packed object for packing or unpacking. Enables syntax simplification.
class Package
{
public:
    /// Default constructor
    Package( )
    {
        data = NULL;
        n = 0;
    }
    /// Copy constructor
    Package( const Package &p )
    {
        data = NULL;
        this->operator=( p );
    }
    /// Construct an object from an existing buffer
    Package( const void* ptr, uint32_t len )
    {
        data = NULL;
        set( ptr, len );
    }
    /// Construct a Package from anything that can be packed
        //template<class T> Package( const T &x )		{ data = NULL; Packer p; p << x; this->data = p.duplicate( this->n ); }
    /// Default destructor
    ~Package( )
    {
        reset( );
    }
    /// Attempt to unpack the object as the specified datatype
    template<class T> T as( )
    {
        T x;
        Unpacker(( const byte* )data,n,0 ) >> x;
        return x;
    }
    /// Convenience syntax for as<> casting
    template<class T> Package& operator>>( T& x )
    {
        x = this->as<T>( );
        return *this;
    }

    template<class T> Package& operator<<( const T& x )
    {
        Packer p;
        p << x;
        this->data = p.duplicate( this->n );
        return *this;
    }

    void set( const void* ptr, uint32_t len )
    {
        reset( );
        if ( !ptr || !len ) return;
        data = malloc( len );
        memcpy( data, ptr, len );
        n = len;
    }

    MSGPACK_TYPE_CODES type( ) const
    {
        if ( !data ) return MSGPACK_NULL;
        return ( MSGPACK_TYPE_CODES )msgpack_unpack_peek_code( *
                ( byte* )data );
    }

    /// Extract a single object from the stream
    friend Unpacker& operator>>( Unpacker &u, Package &obj )
    {
        uint32_t k = u.skip( );
        obj = Package( u.ptr() - k, k );
        return u;
    }
    /// Insert this object into the stream
    friend Packer& operator<<( Packer &p, const Package &obj )
    {
        if ( obj.data == NULL ) p.pack_null( );
        else p.append( obj.data, obj.n );
        return p;
    }

    Package& operator=( const Package& rhs )
    {
        if ( &rhs == this ) return *this;
        this->set( rhs.data, rhs.n );
        return *this;
    }

protected:
    void reset( )
    {
        if ( data ) {
            free( data );
            data = NULL;
        }
        n = 0;
    }
    void *data;
    uint32_t n;
};

} // namespace msgpack

#undef MSGPACK_ASSERT
#endif
