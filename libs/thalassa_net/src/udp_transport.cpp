#include "thalassa/net/udp_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace thalassa::net {

namespace {

std::uint64_t key_of(std::uint32_t ip, std::uint16_t port) noexcept {
    return (static_cast<std::uint64_t>(ip) << 16) | port;
}

void set_error(std::string* error, const std::string& what) {
    if (error != nullptr) *error = what + ": " + std::strerror(errno);
}

}  // namespace

std::unique_ptr<UdpTransport> UdpTransport::create(const UdpConfig& config, std::string* error) {
    std::unique_ptr<UdpTransport> t(new UdpTransport());
    t->max_peers_ = std::max<std::size_t>(config.max_peers, 1);

    t->fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (t->fd_ < 0) {
        set_error(error, "socket");
        return nullptr;
    }
    const int flags = ::fcntl(t->fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(t->fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        set_error(error, "fcntl(O_NONBLOCK)");
        return nullptr;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config.port);
    if (::inet_pton(AF_INET, config.bind_address.c_str(), &addr.sin_addr) != 1) {
        if (error != nullptr) *error = "bad bind address: " + config.bind_address;
        return nullptr;
    }
    if (::bind(t->fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        set_error(error, "bind");
        return nullptr;
    }
    socklen_t len = sizeof(addr);
    if (::getsockname(t->fd_, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
        set_error(error, "getsockname");
        return nullptr;
    }
    t->local_port_ = ntohs(addr.sin_port);
    return t;
}

UdpTransport::~UdpTransport() {
    if (fd_ >= 0) ::close(fd_);
}

PeerId UdpTransport::register_peer(const Address& address) {
    const auto key = key_of(address.ip, address.port);
    if (const auto it = by_address_.find(key); it != by_address_.end()) {
        by_id_[it->second].last_seen = ++clock_;
        return it->second;
    }
    if (by_id_.size() >= max_peers_ && !by_id_.empty()) {
        // Evict the peer that has been silent the longest.
        const auto oldest = std::min_element(by_id_.begin(), by_id_.end(), [](const auto& a, const auto& b) {
            return a.second.last_seen < b.second.last_seen;
        });
        if (oldest != by_id_.end()) {
            by_address_.erase(key_of(oldest->second.address.ip, oldest->second.address.port));
            by_id_.erase(oldest);
            ++stats_.peers_evicted;
        }
    }
    const PeerId id = next_peer_++;
    by_address_.emplace(key, id);
    by_id_.emplace(id, PeerEntry{address, ++clock_});
    return id;
}

PeerId UdpTransport::add_peer(const std::string& host, std::uint16_t port) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    if (::getaddrinfo(host.c_str(), nullptr, &hints, &result) != 0 || result == nullptr) return kInvalidPeer;
    const auto* sin = reinterpret_cast<const sockaddr_in*>(result->ai_addr);
    Address address;
    address.ip = sin->sin_addr.s_addr;
    address.port = htons(port);
    ::freeaddrinfo(result);
    return register_peer(address);
}

std::string UdpTransport::peer_name(PeerId peer) const {
    const auto it = by_id_.find(peer);
    if (it == by_id_.end()) return {};
    in_addr ia{};
    ia.s_addr = it->second.address.ip;
    char buf[INET_ADDRSTRLEN] = {};
    ::inet_ntop(AF_INET, &ia, buf, sizeof(buf));
    return std::string(buf) + ":" + std::to_string(ntohs(it->second.address.port));
}

bool UdpTransport::send(PeerId to, std::span<const std::byte> payload) {
    if (payload.size() > kMaxDatagramBytes) return false;
    const auto it = by_id_.find(to);
    if (it == by_id_.end()) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = it->second.address.ip;
    addr.sin_port = it->second.address.port;
    const ssize_t n = ::sendto(fd_, payload.data(), payload.size(), 0,
                               reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    if (n < 0 || static_cast<std::size_t>(n) != payload.size()) {
        ++stats_.send_failures;
        return false;
    }
    ++stats_.datagrams_sent;
    stats_.bytes_sent += payload.size();
    return true;
}

bool UdpTransport::receive(Datagram& out) {
    for (;;) {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t n = ::recvfrom(fd_, recv_buffer_.data(), recv_buffer_.size(), 0,
                                     reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n < 0) return false;  // EAGAIN (nothing waiting) or a transient error: try next frame
        if (n == 0 || static_cast<std::size_t>(n) > kMaxDatagramBytes) {
            ++stats_.dropped_oversize;
            continue;
        }
        Address address;
        address.ip = from.sin_addr.s_addr;
        address.port = from.sin_port;
        out.peer = register_peer(address);
        out.payload.assign(recv_buffer_.begin(), recv_buffer_.begin() + n);
        ++stats_.datagrams_received;
        stats_.bytes_received += static_cast<std::uint64_t>(n);
        return true;
    }
}

}  // namespace thalassa::net
