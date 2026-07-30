/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace openarm::runtime {
namespace {

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
std::atomic<std::int64_t> fsync_failure_countdown{-1};
std::atomic<std::uint64_t> fsync_failure_mask{0U};
std::atomic<std::uint64_t> fsync_call_index{0U};
#endif

int persistence_fsync(int fd) {
#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
    const std::uint64_t call =
        fsync_call_index.fetch_add(1U, std::memory_order_relaxed);
    const std::uint64_t mask = fsync_failure_mask.load(std::memory_order_relaxed);
    if (call < 64U && (mask & (UINT64_C(1) << call)) != 0U) {
        errno = EIO;
        return -1;
    }
    std::int64_t value = fsync_failure_countdown.load(std::memory_order_relaxed);
    while (value >= 0) {
        if (fsync_failure_countdown.compare_exchange_weak(
                value, value - 1, std::memory_order_relaxed)) {
            if (value == 0) {
                errno = EIO;
                return -1;
            }
            break;
        }
    }
#endif
    return fsync(fd);
}

bool safe_key_id(const char *value) {
    if (value == nullptr) return false;
    const std::size_t size = strnlen(value, OA_RUNTIME_TEXT_CAPACITY);
    if (size == 0U || size >= OA_RUNTIME_TEXT_CAPACITY) return false;
    return std::all_of(value, value + size, [](const char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.' ||
               c == ':';
    });
}

bool safe_file_name(const char *value) {
    if (value == nullptr) return false;
    const std::size_t size = strnlen(value, 201U);
    return size > 0U && size <= 180U && std::strchr(value, '/') == nullptr &&
           std::strncmp(value, ".openarm-runtime-", 17U) != 0 &&
           std::strncmp(value, ".openarm-prior-", 15U) != 0 &&
           std::strcmp(value, ".") != 0 && std::strcmp(value, "..") != 0 &&
           std::all_of(value, value + size, [](const char c) {
               return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
           });
}

bool safe_v2_file_name(const char *value) {
    if (!safe_file_name(value)) return false;
    const std::size_t size = std::strlen(value);
    static constexpr char suffix[] = ".previous";
    return size < sizeof(suffix) - 1U ||
           std::strcmp(value + size - (sizeof(suffix) - 1U), suffix) != 0;
}

bool absolute_directory_path(const char *path, std::string &directory) {
    if (path == nullptr) return false;
    directory = path;
    if (directory.empty() || directory.size() >= PATH_MAX || directory.front() != '/' ||
        (directory.size() > 1U && directory.back() == '/') ||
        directory.find("//") != std::string::npos ||
        directory.find("/./") != std::string::npos ||
        directory.find("/../") != std::string::npos ||
        (directory.size() >= 2U &&
         directory.compare(directory.size() - 2U, 2U, "/.") == 0) ||
        (directory.size() >= 3U &&
         directory.compare(directory.size() - 3U, 3U, "/..") == 0)) {
        return false;
    }
    std::string component;
    for (std::size_t index = 1U; index <= directory.size(); ++index) {
        if (index != directory.size() && directory[index] != '/') continue;
        component = directory.substr(0U, index == 1U ? 1U : index);
        struct stat status{};
        if (lstat(component.c_str(), &status) != 0 || !S_ISDIR(status.st_mode) ||
            S_ISLNK(status.st_mode)) {
            return false;
        }
    }
    return true;
}

int open_absolute_directory(const char *path) {
    if (path == nullptr) return -1;
    const std::size_t size = strnlen(path, PATH_MAX);
    if (size == 0U || size >= PATH_MAX || path[0] != '/' ||
        (size > 1U && path[size - 1U] == '/')) {
        return -1;
    }
    int current = open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (current < 0 || size == 1U) return current;
    std::size_t begin = 1U;
    while (begin < size) {
        const char *const slash = static_cast<const char *>(
            std::memchr(path + begin, '/', size - begin));
        const std::size_t end = slash == nullptr
                                    ? size
                                    : static_cast<std::size_t>(slash - path);
        if (end == begin) {
            close(current);
            return -1;
        }
        const std::string component(path + begin, end - begin);
        if (component == "." || component == "..") {
            close(current);
            return -1;
        }
        const int next = openat(current, component.c_str(),
                                O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        close(current);
        if (next < 0) return -1;
        current = next;
        begin = end + 1U;
    }
    return current;
}

bool absolute_regular_path(const char *path, std::string &directory, std::string &name) {
    if (path == nullptr) return false;
    const std::string value(path);
    if (value.empty() || value.size() >= PATH_MAX || value.front() != '/' ||
        value.back() == '/' || value.find("//") != std::string::npos ||
        value.find("/./") != std::string::npos || value.find("/../") != std::string::npos) {
        return false;
    }
    const std::size_t slash = value.rfind('/');
    directory = slash == 0U ? "/" : value.substr(0U, slash);
    name = value.substr(slash + 1U);
    if (!safe_file_name(name.c_str())) return false;
    std::string checked;
    if (!absolute_directory_path(directory.c_str(), checked)) return false;
    struct stat status{};
    if (lstat(value.c_str(), &status) == 0) {
        return S_ISREG(status.st_mode) && !S_ISLNK(status.st_mode);
    }
    return errno == ENOENT;
}

oa_runtime_status read_file_at(int directory_fd, const char *name, std::string &text) {
    const int fd = openat(directory_fd, name,
                          O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
    if (fd < 0) return OA_RUNTIME_EIO;
    struct stat status{};
    if (fstat(fd, &status) != 0 || !S_ISREG(status.st_mode) || status.st_size <= 0 ||
        status.st_size > 65536) {
        close(fd);
        return OA_RUNTIME_ECORRUPT;
    }
    try {
        text.assign(static_cast<std::size_t>(status.st_size), '\0');
    } catch (...) {
        close(fd);
        return OA_RUNTIME_ENOMEM;
    }
    std::size_t offset = 0U;
    while (offset < text.size()) {
        const ssize_t count = read(fd, text.data() + offset, text.size() - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) {
            close(fd);
            return OA_RUNTIME_EIO;
        }
        offset += static_cast<std::size_t>(count);
    }
    std::uint8_t extra = 0U;
    if (read(fd, &extra, 1U) != 0) {
        close(fd);
        return OA_RUNTIME_ECORRUPT;
    }
    return close(fd) == 0 ? OA_RUNTIME_OK : OA_RUNTIME_EIO;
}

oa_runtime_status read_file(const char *path, std::string &text) {
    std::string directory;
    std::string name;
    if (!absolute_regular_path(path, directory, name)) return OA_RUNTIME_EPERMISSION;
    const int directory_fd =
        open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (directory_fd < 0) return OA_RUNTIME_EIO;
    const oa_runtime_status result = read_file_at(directory_fd, name.c_str(), text);
    close(directory_fd);
    return result;
}

bool constant_time_equal(const std::string &left, const std::string &right) {
    if (left.size() != right.size()) return false;
    std::uint8_t difference = 0U;
    for (std::size_t index = 0U; index < left.size(); ++index) {
        difference = static_cast<std::uint8_t>(
            difference | static_cast<std::uint8_t>(left[index] ^ right[index]));
    }
    return difference == 0U;
}

oa_runtime_status authenticate_text(const PersistenceAuthorityData &authority,
                                    const std::string &text,
                                    std::shared_ptr<ManifestData> &manifest) {
    oa_runtime_status status = parse_manifest(text, manifest);
    if (status != OA_RUNTIME_OK) return status;
    if (manifest->integrity_kind != OA_RUNTIME_INTEGRITY_HMAC_SHA256 ||
        manifest->authentication_version != 1U ||
        manifest->authentication_key_id != authority.authentication_key_id) {
        return OA_RUNTIME_EPERMISSION;
    }
    const std::size_t authentication_offset = text.rfind("hmac-sha256|");
    if (authentication_offset == std::string::npos || authentication_offset == 0U) {
        return OA_RUNTIME_ECORRUPT;
    }
    const std::string expected = hmac_sha256_hex(
        authority.authentication_key, text.substr(0U, authentication_offset));
    if (!constant_time_equal(expected, manifest->authentication_tag)) {
        return OA_RUNTIME_EPERMISSION;
    }
    manifest->authenticated = true;
    manifest->checkpoint_authorized = false;
    manifest->loaded_from_file = true;
    return OA_RUNTIME_OK;
}

std::string slot_authentication_input(const PersistenceAuthorityData &authority,
                                      const char *slot_name,
                                      const std::string &payload) {
    std::string input("OPENARM-RUNTIME-SLOT-V2", 23U);
    input.push_back('\0');
    input += authority.authentication_key_id;
    input.push_back('\0');
    input += slot_name;
    input.push_back('\0');
    input += payload;
    return input;
}

oa_runtime_status authenticate_text_v2(const PersistenceAuthorityData &authority,
                                       const char *slot_name,
                                       const std::string &text,
                                       std::shared_ptr<ManifestData> &manifest) {
    oa_runtime_status status = parse_manifest(text, manifest);
    if (status != OA_RUNTIME_OK) return status;
    if (manifest->integrity_kind != OA_RUNTIME_INTEGRITY_HMAC_SHA256 ||
        manifest->authentication_version != 2U ||
        manifest->authentication_key_id != authority.authentication_key_id) {
        return OA_RUNTIME_EPERMISSION;
    }
    const std::size_t authentication_offset = text.rfind("hmac-sha256-v2|");
    if (authentication_offset == std::string::npos || authentication_offset == 0U) {
        return OA_RUNTIME_ECORRUPT;
    }
    const std::string payload = text.substr(0U, authentication_offset);
    const std::string expected = hmac_sha256_hex(
        authority.authentication_key,
        slot_authentication_input(authority, slot_name, payload));
    if (!constant_time_equal(expected, manifest->authentication_tag)) {
        return OA_RUNTIME_EPERMISSION;
    }
    manifest->authenticated = true;
    manifest->checkpoint_authorized = false;
    manifest->loaded_from_file = true;
    return OA_RUNTIME_OK;
}

oa_runtime_status load_authenticated_at(const PersistenceAuthorityData &authority,
                                        const char *name,
                                        std::shared_ptr<ManifestData> &manifest) {
    try {
        std::string text;
        const oa_runtime_status status = read_file_at(authority.directory_fd, name, text);
        return status == OA_RUNTIME_OK ? authenticate_text(authority, text, manifest) : status;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

oa_runtime_status load_authenticated_at_v2(
    const PersistenceAuthorityData &authority, const char *entry_name,
    const char *slot_name, std::shared_ptr<ManifestData> &manifest) {
    try {
        std::string text;
        const oa_runtime_status status =
            read_file_at(authority.directory_fd, entry_name, text);
        return status == OA_RUNTIME_OK
                   ? authenticate_text_v2(authority, slot_name, text, manifest)
                   : status;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

class DirectoryTransaction {
public:
    explicit DirectoryTransaction(const int directory_fd) {
        fd_ = openat(directory_fd, ".",
                     O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0) return;
        int result = -1;
        do {
            result = flock(fd_, LOCK_EX);
        } while (result != 0 && errno == EINTR);
        locked_ = result == 0;
        if (!locked_) {
            close(fd_);
            fd_ = -1;
        }
    }
    ~DirectoryTransaction() {
        if (fd_ >= 0) close(fd_);
    }
    bool locked() const { return locked_; }

private:
    int fd_{-1};
    bool locked_{};
};

bool lowercase_digest(const char *digest) {
    if (digest == nullptr || strnlen(digest, OA_RUNTIME_DIGEST_CAPACITY) != 64U) {
        return false;
    }
    return std::all_of(digest, digest + 64U, [](const char value) {
        return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
    });
}

oa_runtime_status checkpoint_status(
    const oa_runtime_persistence_checkpoint *checkpoint) {
    if (checkpoint == nullptr || checkpoint->struct_size < sizeof(*checkpoint) ||
        checkpoint->abi_version != OA_RUNTIME_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    if (checkpoint->revision == 0U) {
        return std::all_of(
                   checkpoint->content_sha256,
                   checkpoint->content_sha256 + OA_RUNTIME_DIGEST_CAPACITY,
                   [](const char value) { return value == '\0'; })
                   ? OA_RUNTIME_OK
                   : OA_RUNTIME_EINVAL;
    }
    return lowercase_digest(checkpoint->content_sha256) ? OA_RUNTIME_OK
                                                         : OA_RUNTIME_EINVAL;
}

void clear_checkpoint(oa_runtime_persistence_checkpoint *checkpoint) {
    if (checkpoint != nullptr) runtime_init(*checkpoint);
}

void fill_checkpoint(const ManifestData &manifest,
                     oa_runtime_persistence_checkpoint &checkpoint) {
    runtime_init(checkpoint);
    checkpoint.revision = manifest.config.manifest_revision;
    std::snprintf(checkpoint.content_sha256, sizeof(checkpoint.content_sha256), "%s",
                  manifest.content_digest.c_str());
}

bool exact_checkpoint(const ManifestData &manifest,
                      const oa_runtime_persistence_checkpoint &checkpoint) {
    return manifest.config.manifest_revision == checkpoint.revision &&
           manifest.content_digest == checkpoint.content_sha256;
}

oa_runtime_status bind_v2_call(PersistenceAuthorityData &authority,
                               const char *file_name,
                               const oa_runtime_persistence_checkpoint &checkpoint) {
    if (authority.protocol_version != 2U) return OA_RUNTIME_EUNSUPPORTED;
    if (authority.poisoned) return OA_RUNTIME_EDURABILITY;
    if (authority.bound_file_name.empty()) {
        authority.bound_file_name = file_name;
    } else if (authority.bound_file_name != file_name) {
        return OA_RUNTIME_EIDENTITY;
    }
    if (checkpoint.revision < authority.trusted_revision) return OA_RUNTIME_ESTALE;
    if (checkpoint.revision == authority.trusted_revision &&
        std::string(checkpoint.content_sha256) != authority.trusted_digest) {
        return OA_RUNTIME_ESTALE;
    }
    return OA_RUNTIME_OK;
}

oa_runtime_status satisfies_floor(
    const ManifestData &manifest,
    const oa_runtime_persistence_checkpoint &checkpoint) {
    if (manifest.config.manifest_revision < checkpoint.revision) {
        return OA_RUNTIME_ESTALE;
    }
    if (manifest.config.manifest_revision == checkpoint.revision &&
        manifest.content_digest != checkpoint.content_sha256) {
        return OA_RUNTIME_ESTALE;
    }
    return OA_RUNTIME_OK;
}

oa_runtime_status refresh_revision_floor(PersistenceAuthorityData &authority) {
    const int scan_fd = dup(authority.directory_fd);
    if (scan_fd < 0) return OA_RUNTIME_EIO;
    DIR *const directory = fdopendir(scan_fd);
    if (directory == nullptr) {
        close(scan_fd);
        return OA_RUNTIME_EIO;
    }
    std::uint64_t maximum_revision = authority.accepted_revision_floor;
    std::string maximum_digest = authority.accepted_revision_digest;
    int read_error = 0;
    for (;;) {
        errno = 0;
        const dirent *const entry = readdir(directory);
        if (entry == nullptr) {
            read_error = errno;
            break;
        }
        if (!safe_file_name(entry->d_name)) continue;
        std::shared_ptr<ManifestData> candidate;
        if (load_authenticated_at(authority, entry->d_name, candidate) != OA_RUNTIME_OK) {
            continue;
        }
        const std::uint64_t revision = candidate->config.manifest_revision;
        if (revision > maximum_revision) {
            maximum_revision = revision;
            maximum_digest = candidate->content_digest;
        } else if (revision == maximum_revision && !maximum_digest.empty() &&
                   candidate->content_digest != maximum_digest) {
            closedir(directory);
            return OA_RUNTIME_ESTALE;
        } else if (revision == maximum_revision && maximum_digest.empty()) {
            maximum_digest = candidate->content_digest;
        }
    }
    if (closedir(directory) != 0 || read_error != 0) return OA_RUNTIME_EIO;
    authority.accepted_revision_floor = maximum_revision;
    authority.accepted_revision_digest = std::move(maximum_digest);
    return OA_RUNTIME_OK;
}

bool target_is_regular_or_absent(int directory_fd, const char *name, bool &exists) {
    struct stat status{};
    if (fstatat(directory_fd, name, &status, AT_SYMLINK_NOFOLLOW) == 0) {
        exists = true;
        return S_ISREG(status.st_mode) && !S_ISLNK(status.st_mode);
    }
    exists = false;
    return errno == ENOENT;
}

oa_runtime_status accept_revision(PersistenceAuthorityData &authority,
                                  const ManifestData &manifest,
                                  const char *name,
                                  const bool advance) {
    const std::uint64_t revision = manifest.config.manifest_revision;
    if (revision < authority.accepted_revision_floor) return OA_RUNTIME_ESTALE;
    if (revision == authority.accepted_revision_floor &&
        !authority.accepted_revision_digest.empty() &&
        manifest.content_digest != authority.accepted_revision_digest) {
        return OA_RUNTIME_ESTALE;
    }
    const auto found = authority.accepted_artifacts.find(name);
    if (found != authority.accepted_artifacts.end()) {
        if (revision < found->second.revision) return OA_RUNTIME_ESTALE;
        if (revision == found->second.revision &&
            manifest.content_digest != found->second.content_digest) {
            return OA_RUNTIME_ESTALE;
        }
    }
    if (advance) {
        if (revision > authority.accepted_revision_floor ||
            authority.accepted_revision_digest.empty()) {
            authority.accepted_revision_floor = revision;
            authority.accepted_revision_digest = manifest.content_digest;
        }
        PersistenceAuthorityData::AcceptedArtifact &accepted =
            authority.accepted_artifacts[name];
        if (revision >= accepted.revision) {
            accepted.revision = revision;
            accepted.content_digest = manifest.content_digest;
        }
    }
    return OA_RUNTIME_OK;
}

oa_runtime_status build_signed_v2(const PersistenceAuthorityData &authority,
                                  const ManifestData &source,
                                  const char *slot_name,
                                  ManifestData &copy,
                                  std::string &contents) {
    copy = source;
    copy.content_digest = sha256_hex(manifest_canonical(copy, false));
    copy.integrity_kind = OA_RUNTIME_INTEGRITY_HMAC_SHA256;
    copy.authenticated = true;
    copy.checkpoint_authorized = true;
    copy.authentication_version = 2U;
    copy.authentication_key_id = authority.authentication_key_id;
    const std::string payload = manifest_canonical(copy, true);
    copy.authentication_tag = hmac_sha256_hex(
        authority.authentication_key,
        slot_authentication_input(authority, slot_name, payload));
    contents = payload + "hmac-sha256-v2|" + copy.authentication_key_id + '|' +
               copy.authentication_tag + '\n';
    return OA_RUNTIME_OK;
}

oa_runtime_status sync_existing(int directory_fd, const char *file_name) {
    const int fd = openat(directory_fd, file_name,
                          O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) return OA_RUNTIME_EIO;
    struct stat status{};
    const bool regular = fstat(fd, &status) == 0 && S_ISREG(status.st_mode);
    const bool synced = regular && persistence_fsync(fd) == 0;
    const bool closed = close(fd) == 0;
    if (!synced || !closed) return OA_RUNTIME_EIO;
    return persistence_fsync(directory_fd) == 0 ? OA_RUNTIME_OK : OA_RUNTIME_EIO;
}

oa_runtime_status install_signed_v2(PersistenceAuthorityData &authority,
                                    const char *file_name,
                                    const ManifestData &copy,
                                    const std::string &contents,
                                    const bool target_exists) {
    static std::atomic<std::uint64_t> sequence{UINT64_C(1000000)};
    const std::string suffix = std::to_string(getpid()) + "-" +
        std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed));
    const std::string temporary = ".openarm-runtime-" + suffix;
    const std::string backup_temporary = ".openarm-prior-" + suffix;
    const std::string previous_name = std::string(file_name) + ".previous";
    int fd = openat(authority.directory_fd, temporary.c_str(),
                    O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) return OA_RUNTIME_EIO;
    std::size_t offset = 0U;
    bool success = true;
    while (offset < contents.size()) {
        const ssize_t count = write(fd, contents.data() + offset,
                                    contents.size() - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) {
            success = false;
            break;
        }
        offset += static_cast<std::size_t>(count);
    }
    if (success && persistence_fsync(fd) != 0) success = false;
    if (close(fd) != 0) success = false;
    if (!success) {
        unlinkat(authority.directory_fd, temporary.c_str(), 0);
        return OA_RUNTIME_EIO;
    }

    if (target_exists) {
        if (linkat(authority.directory_fd, file_name,
                   authority.directory_fd, backup_temporary.c_str(), 0) != 0 ||
            renameat(authority.directory_fd, backup_temporary.c_str(),
                     authority.directory_fd, previous_name.c_str()) != 0 ||
            persistence_fsync(authority.directory_fd) != 0) {
            unlinkat(authority.directory_fd, backup_temporary.c_str(), 0);
            unlinkat(authority.directory_fd, temporary.c_str(), 0);
            return OA_RUNTIME_EIO;
        }
    }

    if (renameat(authority.directory_fd, temporary.c_str(),
                 authority.directory_fd, file_name) != 0) {
        unlinkat(authority.directory_fd, temporary.c_str(), 0);
        return OA_RUNTIME_EIO;
    }
    std::shared_ptr<ManifestData> verified;
    const oa_runtime_status verification =
        load_authenticated_at_v2(authority, file_name, file_name, verified);
    if (verification != OA_RUNTIME_OK ||
        verified->content_digest != copy.content_digest ||
        verified->config.manifest_revision != copy.config.manifest_revision) {
        const bool rolled_back = target_exists
            ? renameat(authority.directory_fd, previous_name.c_str(),
                       authority.directory_fd, file_name) == 0
            : unlinkat(authority.directory_fd, file_name, 0) == 0;
        if (!rolled_back || persistence_fsync(authority.directory_fd) != 0) {
            authority.poisoned = true;
            return OA_RUNTIME_EDURABILITY;
        }
        return verification == OA_RUNTIME_OK ? OA_RUNTIME_EIO : verification;
    }
    if (persistence_fsync(authority.directory_fd) != 0) {
        const bool rolled_back = target_exists
            ? renameat(authority.directory_fd, previous_name.c_str(),
                       authority.directory_fd, file_name) == 0
            : unlinkat(authority.directory_fd, file_name, 0) == 0;
        if (rolled_back && persistence_fsync(authority.directory_fd) == 0) {
            return OA_RUNTIME_EIO;
        }
        authority.poisoned = true;
        return OA_RUNTIME_EDURABILITY;
    }
    return OA_RUNTIME_OK;
}

} // namespace
} // namespace openarm::runtime

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
extern "C" void oa_runtime_test_fail_fsync_after(std::int64_t countdown) {
    openarm::runtime::fsync_failure_countdown.store(countdown,
                                                    std::memory_order_relaxed);
}

extern "C" void oa_runtime_test_fail_fsync_mask(std::uint64_t mask) {
    openarm::runtime::fsync_call_index.store(0U, std::memory_order_relaxed);
    openarm::runtime::fsync_failure_mask.store(mask, std::memory_order_relaxed);
}
#endif

extern "C" oa_runtime_status oa_runtime_manifest_load(
    const char *absolute_path, oa_runtime_manifest **out_manifest) {
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    try {
        std::string text;
        oa_runtime_status status = openarm::runtime::read_file(absolute_path, text);
        if (status != OA_RUNTIME_OK) return status;
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        status = openarm::runtime::parse_manifest(text, manifest);
        if (status != OA_RUNTIME_OK) return status;
        manifest->authenticated = false;
        manifest->loaded_from_file = true;
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_preview_file(
    const char *absolute_path, oa_runtime_manifest_preview *out_preview) {
    if (!openarm::runtime::output_valid(out_preview)) return OA_RUNTIME_EABI;
    try {
        openarm::runtime::allocation_checkpoint();
        std::string text;
        oa_runtime_status status = openarm::runtime::read_file(absolute_path, text);
        if (status != OA_RUNTIME_OK) return status;
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        status = openarm::runtime::parse_manifest(text, manifest);
        if (status != OA_RUNTIME_OK) return status;
        oa_runtime_manifest_preview result{};
        openarm::runtime::runtime_init(result);
        result.valid = 1U;
        result.would_be_armable = manifest->state == OA_RUNTIME_MANIFEST_ARMABLE ? 1U : 0U;
        result.base_revision = manifest->config.manifest_revision;
        result.result_revision = manifest->config.manifest_revision;
        result.validation_status = OA_RUNTIME_OK;
        *out_preview = result;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_persistence_authority_create(
    const char *absolute_directory,
    const std::uint8_t authentication_key[OA_RUNTIME_PERSISTENCE_KEY_BYTES],
    const char *authentication_key_id,
    oa_runtime_persistence_authority **out_authority) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_authority == nullptr) return OA_RUNTIME_EINVAL;
    *out_authority = nullptr;
    if (authentication_key == nullptr ||
        !openarm::runtime::safe_key_id(authentication_key_id)) {
        return OA_RUNTIME_EINVAL;
    }
    if (std::all_of(authentication_key,
                    authentication_key + OA_RUNTIME_PERSISTENCE_KEY_BYTES,
                    [](const std::uint8_t value) { return value == 0U; })) {
        return OA_RUNTIME_EPERMISSION;
    }
    try {
        auto authority = std::make_shared<openarm::runtime::PersistenceAuthorityData>();
        authority->directory_fd =
            openarm::runtime::open_absolute_directory(absolute_directory);
        if (authority->directory_fd < 0) return OA_RUNTIME_EPERMISSION;
        std::copy_n(authentication_key, OA_RUNTIME_PERSISTENCE_KEY_BYTES,
                    authority->authentication_key.begin());
        authority->authentication_key_id = authentication_key_id;
        oa_runtime_persistence_authority *const handle =
            openarm::runtime::persistence_authorities.insert(authority);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_authority = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_persistence_authority_open_v2(
    const char *absolute_directory,
    const std::uint8_t authentication_key[OA_RUNTIME_PERSISTENCE_KEY_BYTES],
    const char *authentication_key_id,
    const oa_runtime_persistence_checkpoint *trusted_checkpoint,
    oa_runtime_persistence_authority **out_authority) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_authority == nullptr) return OA_RUNTIME_EINVAL;
    *out_authority = nullptr;
    const oa_runtime_status checkpoint_status =
        openarm::runtime::checkpoint_status(trusted_checkpoint);
    if (checkpoint_status != OA_RUNTIME_OK) return checkpoint_status;
    if (authentication_key == nullptr ||
        !openarm::runtime::safe_key_id(authentication_key_id)) {
        return OA_RUNTIME_EINVAL;
    }
    if (std::all_of(authentication_key,
                    authentication_key + OA_RUNTIME_PERSISTENCE_KEY_BYTES,
                    [](const std::uint8_t value) { return value == 0U; })) {
        return OA_RUNTIME_EPERMISSION;
    }
    try {
        auto authority = std::make_shared<openarm::runtime::PersistenceAuthorityData>();
        authority->directory_fd =
            openarm::runtime::open_absolute_directory(absolute_directory);
        if (authority->directory_fd < 0) return OA_RUNTIME_EPERMISSION;
        std::copy_n(authentication_key, OA_RUNTIME_PERSISTENCE_KEY_BYTES,
                    authority->authentication_key.begin());
        authority->authentication_key_id = authentication_key_id;
        authority->protocol_version = 2U;
        authority->trusted_revision = trusted_checkpoint->revision;
        authority->trusted_digest = trusted_checkpoint->content_sha256;
        oa_runtime_persistence_authority *const handle =
            openarm::runtime::persistence_authorities.insert(authority);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_authority = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" void oa_runtime_persistence_authority_destroy(
    oa_runtime_persistence_authority *authority) {
    if (!openarm::runtime::process_guard_ok()) return;
    openarm::runtime::persistence_authorities.erase(authority);
}

extern "C" oa_runtime_status oa_runtime_manifest_load_authenticated(
    const oa_runtime_persistence_authority *authority, const char *file_name,
    oa_runtime_manifest **out_manifest) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    const auto pinned = openarm::runtime::persistence_authorities.pin(authority);
    if (!pinned || !openarm::runtime::safe_file_name(file_name)) return OA_RUNTIME_EINVAL;
    try {
        std::lock_guard<std::mutex> authority_lock(pinned->mutex);
        if (pinned->protocol_version != 1U) return OA_RUNTIME_EUNSUPPORTED;
        openarm::runtime::DirectoryTransaction transaction(pinned->directory_fd);
        if (!transaction.locked()) return OA_RUNTIME_EIO;
        oa_runtime_status status =
            openarm::runtime::refresh_revision_floor(*pinned);
        if (status != OA_RUNTIME_OK) return status;
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        status = openarm::runtime::load_authenticated_at(*pinned, file_name, manifest);
        if (status != OA_RUNTIME_OK) return status;
        const std::size_t name_size = std::strlen(file_name);
        static constexpr char previous_suffix[] = ".previous";
        if (name_size >= sizeof(previous_suffix) - 1U &&
            std::strcmp(file_name + name_size - (sizeof(previous_suffix) - 1U),
                        previous_suffix) == 0) {
            return OA_RUNTIME_ESTALE;
        }
        status = openarm::runtime::accept_revision(
            *pinned, *manifest, file_name, true);
        if (status != OA_RUNTIME_OK) return status;
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_save(
    const oa_runtime_manifest *manifest,
    const oa_runtime_persistence_authority *authority,
    const char *file_name) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    const auto authority_data = openarm::runtime::persistence_authorities.pin(authority);
    if (!manifest_data || !authority_data ||
        !openarm::runtime::safe_file_name(file_name)) {
        return OA_RUNTIME_EINVAL;
    }
    try {
        std::lock_guard<std::mutex> authority_lock(authority_data->mutex);
        if (authority_data->protocol_version != 1U) {
            return OA_RUNTIME_EUNSUPPORTED;
        }
        openarm::runtime::DirectoryTransaction transaction(
            authority_data->directory_fd);
        if (!transaction.locked()) return OA_RUNTIME_EIO;
        oa_runtime_status floor_status =
            openarm::runtime::refresh_revision_floor(*authority_data);
        if (floor_status != OA_RUNTIME_OK) return floor_status;
        bool target_exists = false;
        if (!openarm::runtime::target_is_regular_or_absent(
                authority_data->directory_fd, file_name, target_exists)) {
            return OA_RUNTIME_EPERMISSION;
        }

        openarm::runtime::ManifestData copy = *manifest_data;
        copy.content_digest = openarm::runtime::sha256_hex(
            openarm::runtime::manifest_canonical(copy, false));
        copy.integrity_kind = OA_RUNTIME_INTEGRITY_HMAC_SHA256;
        copy.authenticated = true;
        copy.authentication_key_id = authority_data->authentication_key_id;
        const std::string signed_content =
            openarm::runtime::manifest_canonical(copy, true);
        copy.authentication_tag = openarm::runtime::hmac_sha256_hex(
            authority_data->authentication_key, signed_content);
        const std::string contents = signed_content + "hmac-sha256|" +
            copy.authentication_key_id + '|' + copy.authentication_tag + '\n';

        floor_status = openarm::runtime::accept_revision(
            *authority_data, copy, file_name, false);
        if (floor_status != OA_RUNTIME_OK) return floor_status;

        if (target_exists) {
            std::shared_ptr<openarm::runtime::ManifestData> previous;
            const oa_runtime_status existing_status =
                openarm::runtime::load_authenticated_at(*authority_data, file_name, previous);
            if (existing_status != OA_RUNTIME_OK) return existing_status;
            if (previous->config.manifest_revision > copy.config.manifest_revision ||
                (previous->config.manifest_revision == copy.config.manifest_revision &&
                 previous->content_digest != copy.content_digest)) {
                return OA_RUNTIME_ESTALE;
            }
            if (previous->config.manifest_revision == copy.config.manifest_revision &&
                previous->content_digest == copy.content_digest) {
                (void)openarm::runtime::accept_revision(
                    *authority_data, copy, file_name, true);
                return OA_RUNTIME_OK;
            }
        }

        static std::atomic<std::uint64_t> sequence{1U};
        const std::string suffix = std::to_string(getpid()) + "-" +
            std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed));
        const std::string temporary = ".openarm-runtime-" + suffix;
        const std::string backup_temporary = ".openarm-prior-" + suffix;
        const std::string previous_name = std::string(file_name) + ".previous";
        int fd = openat(authority_data->directory_fd, temporary.c_str(),
                        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0) return OA_RUNTIME_EIO;
        std::size_t offset = 0U;
        bool success = true;
        while (offset < contents.size()) {
            const ssize_t count = write(fd, contents.data() + offset,
                                        contents.size() - offset);
            if (count <= 0) {
                success = false;
                break;
            }
            offset += static_cast<std::size_t>(count);
        }
        if (success && openarm::runtime::persistence_fsync(fd) != 0) success = false;
        if (close(fd) != 0) success = false;
        if (!success) {
            unlinkat(authority_data->directory_fd, temporary.c_str(), 0);
            return OA_RUNTIME_EIO;
        }

        if (target_exists) {
            if (linkat(authority_data->directory_fd, file_name,
                       authority_data->directory_fd, backup_temporary.c_str(), 0) != 0 ||
                renameat(authority_data->directory_fd, backup_temporary.c_str(),
                         authority_data->directory_fd, previous_name.c_str()) != 0 ||
                openarm::runtime::persistence_fsync(authority_data->directory_fd) != 0) {
                unlinkat(authority_data->directory_fd, backup_temporary.c_str(), 0);
                unlinkat(authority_data->directory_fd, temporary.c_str(), 0);
                return OA_RUNTIME_EIO;
            }
        }

        if (renameat(authority_data->directory_fd, temporary.c_str(),
                     authority_data->directory_fd, file_name) != 0) {
            unlinkat(authority_data->directory_fd, temporary.c_str(), 0);
            return OA_RUNTIME_EIO;
        }
        std::shared_ptr<openarm::runtime::ManifestData> verified;
        const oa_runtime_status verification =
            openarm::runtime::load_authenticated_at(*authority_data, file_name, verified);
        if (verification != OA_RUNTIME_OK ||
            verified->content_digest != copy.content_digest ||
            verified->config.manifest_revision != copy.config.manifest_revision) {
            const bool rolled_back = target_exists
                ? renameat(authority_data->directory_fd, previous_name.c_str(),
                           authority_data->directory_fd, file_name) == 0
                : unlinkat(authority_data->directory_fd, file_name, 0) == 0;
            if (!rolled_back ||
                openarm::runtime::persistence_fsync(authority_data->directory_fd) != 0) {
                (void)openarm::runtime::accept_revision(
                    *authority_data, copy, file_name, true);
                return OA_RUNTIME_EDURABILITY;
            }
            return verification == OA_RUNTIME_OK ? OA_RUNTIME_EIO : verification;
        }
        if (openarm::runtime::persistence_fsync(authority_data->directory_fd) != 0) {
            const bool rolled_back = target_exists
                ? renameat(authority_data->directory_fd, previous_name.c_str(),
                           authority_data->directory_fd, file_name) == 0
                : unlinkat(authority_data->directory_fd, file_name, 0) == 0;
            if (rolled_back &&
                openarm::runtime::persistence_fsync(authority_data->directory_fd) == 0) {
                return OA_RUNTIME_EIO;
            }
            (void)openarm::runtime::accept_revision(
                *authority_data, copy, file_name, true);
            return OA_RUNTIME_EDURABILITY;
        }
        (void)openarm::runtime::accept_revision(
            *authority_data, copy, file_name, true);
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_load_authenticated_v2(
    const oa_runtime_persistence_authority *authority, const char *file_name,
    const oa_runtime_persistence_checkpoint *trusted_checkpoint,
    oa_runtime_manifest **out_manifest,
    oa_runtime_persistence_checkpoint *out_observed_checkpoint) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    if (!openarm::runtime::output_valid(out_observed_checkpoint)) {
        return OA_RUNTIME_EABI;
    }
    openarm::runtime::clear_checkpoint(out_observed_checkpoint);
    const oa_runtime_status checkpoint_status =
        openarm::runtime::checkpoint_status(trusted_checkpoint);
    if (checkpoint_status != OA_RUNTIME_OK) return checkpoint_status;
    if (trusted_checkpoint->revision == 0U) return OA_RUNTIME_EPERMISSION;
    const auto pinned = openarm::runtime::persistence_authorities.pin(authority);
    if (!pinned || !openarm::runtime::safe_v2_file_name(file_name)) {
        return OA_RUNTIME_EINVAL;
    }
    try {
        std::lock_guard<std::mutex> authority_lock(pinned->mutex);
        oa_runtime_status status = openarm::runtime::bind_v2_call(
            *pinned, file_name, *trusted_checkpoint);
        if (status != OA_RUNTIME_OK) return status;
        openarm::runtime::DirectoryTransaction transaction(pinned->directory_fd);
        if (!transaction.locked()) return OA_RUNTIME_EIO;
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        status = openarm::runtime::load_authenticated_at_v2(
            *pinned, file_name, file_name, manifest);
        if (status != OA_RUNTIME_OK) return status;
        status = openarm::runtime::satisfies_floor(*manifest, *trusted_checkpoint);
        if (status != OA_RUNTIME_OK) return status;
        manifest->checkpoint_authorized = true;
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        oa_runtime_persistence_checkpoint observed{};
        openarm::runtime::fill_checkpoint(*manifest, observed);
        pinned->trusted_revision = observed.revision;
        pinned->trusted_digest = observed.content_sha256;
        *out_observed_checkpoint = observed;
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_save_v2(
    const oa_runtime_manifest *manifest,
    const oa_runtime_persistence_authority *authority,
    const char *file_name,
    const oa_runtime_persistence_checkpoint *expected_current_checkpoint,
    oa_runtime_persistence_checkpoint *out_committed_checkpoint) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (!openarm::runtime::output_valid(out_committed_checkpoint)) {
        return OA_RUNTIME_EABI;
    }
    openarm::runtime::clear_checkpoint(out_committed_checkpoint);
    const oa_runtime_status checkpoint_status =
        openarm::runtime::checkpoint_status(expected_current_checkpoint);
    if (checkpoint_status != OA_RUNTIME_OK) return checkpoint_status;
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    const auto authority_data =
        openarm::runtime::persistence_authorities.pin(authority);
    if (!manifest_data || !authority_data ||
        !openarm::runtime::safe_v2_file_name(file_name)) {
        return OA_RUNTIME_EINVAL;
    }
    try {
        std::lock_guard<std::mutex> authority_lock(authority_data->mutex);
        oa_runtime_status status = openarm::runtime::bind_v2_call(
            *authority_data, file_name, *expected_current_checkpoint);
        if (status != OA_RUNTIME_OK) return status;
        openarm::runtime::DirectoryTransaction transaction(
            authority_data->directory_fd);
        if (!transaction.locked()) return OA_RUNTIME_EIO;
        bool target_exists = false;
        if (!openarm::runtime::target_is_regular_or_absent(
                authority_data->directory_fd, file_name, target_exists)) {
            return OA_RUNTIME_EPERMISSION;
        }
        std::shared_ptr<openarm::runtime::ManifestData> current;
        if (target_exists) {
            status = openarm::runtime::load_authenticated_at_v2(
                *authority_data, file_name, file_name, current);
            if (status != OA_RUNTIME_OK) return status;
            if (!openarm::runtime::exact_checkpoint(
                    *current, *expected_current_checkpoint)) {
                return OA_RUNTIME_ESTALE;
            }
        } else if (expected_current_checkpoint->revision != 0U) {
            return OA_RUNTIME_ESTALE;
        }

        openarm::runtime::ManifestData copy{};
        std::string contents;
        status = openarm::runtime::build_signed_v2(
            *authority_data, *manifest_data, file_name, copy, contents);
        if (status != OA_RUNTIME_OK) return status;
        if (target_exists &&
            copy.config.manifest_revision == expected_current_checkpoint->revision &&
            copy.content_digest == expected_current_checkpoint->content_sha256) {
            status = openarm::runtime::sync_existing(
                authority_data->directory_fd, file_name);
            if (status != OA_RUNTIME_OK) return status;
        } else {
            if (copy.config.manifest_revision <=
                expected_current_checkpoint->revision) {
                return OA_RUNTIME_ESTALE;
            }
            status = openarm::runtime::install_signed_v2(
                *authority_data, file_name, copy, contents, target_exists);
            if (status != OA_RUNTIME_OK) return status;
        }
        oa_runtime_persistence_checkpoint committed{};
        openarm::runtime::fill_checkpoint(copy, committed);
        authority_data->trusted_revision = committed.revision;
        authority_data->trusted_digest = committed.content_sha256;
        *out_committed_checkpoint = committed;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_recover_v2(
    const oa_runtime_persistence_authority *authority, const char *file_name,
    const oa_runtime_persistence_checkpoint *trusted_checkpoint,
    oa_runtime_manifest **out_manifest,
    oa_runtime_persistence_checkpoint *out_observed_checkpoint) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    if (!openarm::runtime::output_valid(out_observed_checkpoint)) {
        return OA_RUNTIME_EABI;
    }
    openarm::runtime::clear_checkpoint(out_observed_checkpoint);
    const oa_runtime_status checkpoint_status =
        openarm::runtime::checkpoint_status(trusted_checkpoint);
    if (checkpoint_status != OA_RUNTIME_OK) return checkpoint_status;
    if (trusted_checkpoint->revision == 0U) return OA_RUNTIME_EPERMISSION;
    const auto pinned = openarm::runtime::persistence_authorities.pin(authority);
    if (!pinned || !openarm::runtime::safe_v2_file_name(file_name)) {
        return OA_RUNTIME_EINVAL;
    }
    try {
        std::lock_guard<std::mutex> authority_lock(pinned->mutex);
        if (pinned->protocol_version != 2U) return OA_RUNTIME_EUNSUPPORTED;
        if (pinned->bound_file_name.empty()) {
            pinned->bound_file_name = file_name;
        } else if (pinned->bound_file_name != file_name) {
            return OA_RUNTIME_EIDENTITY;
        }
        if (trusted_checkpoint->revision < pinned->trusted_revision ||
            (trusted_checkpoint->revision == pinned->trusted_revision &&
             std::string(trusted_checkpoint->content_sha256) !=
                 pinned->trusted_digest)) {
            return pinned->poisoned ? OA_RUNTIME_EDURABILITY : OA_RUNTIME_ESTALE;
        }
        if (pinned->poisoned &&
            (trusted_checkpoint->revision != pinned->trusted_revision ||
             std::string(trusted_checkpoint->content_sha256) !=
                 pinned->trusted_digest)) {
            return OA_RUNTIME_EDURABILITY;
        }
        openarm::runtime::DirectoryTransaction transaction(pinned->directory_fd);
        if (!transaction.locked()) return OA_RUNTIME_EIO;
        const std::string previous_name = std::string(file_name) + ".previous";
        std::shared_ptr<openarm::runtime::ManifestData> current;
        std::shared_ptr<openarm::runtime::ManifestData> previous;
        const oa_runtime_status current_status =
            openarm::runtime::load_authenticated_at_v2(
                *pinned, file_name, file_name, current);
        const oa_runtime_status previous_status =
            openarm::runtime::load_authenticated_at_v2(
                *pinned, previous_name.c_str(), file_name, previous);
        const bool current_accepted = current_status == OA_RUNTIME_OK &&
            openarm::runtime::satisfies_floor(
                *current, *trusted_checkpoint) == OA_RUNTIME_OK;
        const bool previous_accepted = previous_status == OA_RUNTIME_OK &&
            openarm::runtime::satisfies_floor(
                *previous, *trusted_checkpoint) == OA_RUNTIME_OK;
        std::shared_ptr<openarm::runtime::ManifestData> selected;
        bool selected_previous = false;
        if (current_accepted) selected = current;
        if (previous_accepted &&
            (!selected || previous->config.manifest_revision >
                              selected->config.manifest_revision)) {
            selected = previous;
            selected_previous = true;
        } else if (previous_accepted && selected &&
                   previous->config.manifest_revision ==
                       selected->config.manifest_revision &&
                   previous->content_digest != selected->content_digest) {
            return OA_RUNTIME_ESTALE;
        }
        if (!selected) {
            if (current_status == OA_RUNTIME_OK || previous_status == OA_RUNTIME_OK) {
                return OA_RUNTIME_ESTALE;
            }
            return current_status != OA_RUNTIME_EIO ? current_status
                                                     : previous_status;
        }
        if (selected_previous) {
            bool current_exists = false;
            if (!openarm::runtime::target_is_regular_or_absent(
                    pinned->directory_fd, file_name, current_exists)) {
                return OA_RUNTIME_EPERMISSION;
            }
            static std::atomic<std::uint64_t> recovery_sequence{1U};
            const std::string temporary = ".openarm-runtime-recovery-" +
                std::to_string(getpid()) + "-" +
                std::to_string(recovery_sequence.fetch_add(
                    1U, std::memory_order_relaxed));
            if (linkat(pinned->directory_fd, previous_name.c_str(),
                       pinned->directory_fd, temporary.c_str(), 0) != 0 ||
                renameat(pinned->directory_fd, temporary.c_str(),
                         pinned->directory_fd, file_name) != 0 ||
                openarm::runtime::persistence_fsync(pinned->directory_fd) != 0) {
                unlinkat(pinned->directory_fd, temporary.c_str(), 0);
                pinned->poisoned = true;
                return OA_RUNTIME_EDURABILITY;
            }
            std::shared_ptr<openarm::runtime::ManifestData> verified;
            const oa_runtime_status verification =
                openarm::runtime::load_authenticated_at_v2(
                    *pinned, file_name, file_name, verified);
            if (verification != OA_RUNTIME_OK ||
                verified->content_digest != selected->content_digest ||
                verified->config.manifest_revision !=
                    selected->config.manifest_revision) {
                pinned->poisoned = true;
                return OA_RUNTIME_EDURABILITY;
            }
            selected = std::move(verified);
        } else {
            const oa_runtime_status sync_status = openarm::runtime::sync_existing(
                pinned->directory_fd, file_name);
            if (sync_status != OA_RUNTIME_OK) {
                if (pinned->poisoned) return OA_RUNTIME_EDURABILITY;
                return sync_status;
            }
        }
        selected->checkpoint_authorized = true;
        oa_runtime_manifest *const handle =
            openarm::runtime::manifests.insert(selected);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        oa_runtime_persistence_checkpoint observed{};
        openarm::runtime::fill_checkpoint(*selected, observed);
        pinned->trusted_revision = observed.revision;
        pinned->trusted_digest = observed.content_sha256;
        pinned->poisoned = false;
        *out_observed_checkpoint = observed;
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}
