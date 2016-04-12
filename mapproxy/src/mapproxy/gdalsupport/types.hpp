#ifndef mapproxy_gdalsupport_types_hpp_included_
#define mapproxy_gdalsupport_types_hpp_included_

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/weak_ptr.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/interprocess/indexes/flat_map_index.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace bi = boost::interprocess;

typedef bi::basic_managed_external_buffer<
    char
    , bi::rbtree_best_fit<bi::mutex_family, void*>
    , bi::flat_map_index> ManagedBuffer;

typedef ManagedBuffer::segment_manager SegmentManager;

typedef bi::allocator<void, SegmentManager> Allocator;
typedef bi::deleter<void, SegmentManager> Deleter;
typedef bi::shared_ptr<cv::Mat, Allocator, Deleter> MatPointer;

typedef bi::basic_string<
    char, std::char_traits<char>
    , bi::allocator<char, SegmentManager>
    > String;

typedef bi::scoped_lock<bi::interprocess_mutex> Lock;

#endif // mapproxy_gdalsupport_types_hpp_included_
