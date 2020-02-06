add_library(sockpp
        src/acceptor.cpp
        src/connector.cpp
        src/datagram_socket.cpp
        src/exception.cpp
        src/inet_address.cpp
        src/inet6_address.cpp
        src/socket.cpp
        src/stream_socket.cpp
        src/unix/unix_address.cpp
        )

target_include_directories(sockpp PUBLIC include)

