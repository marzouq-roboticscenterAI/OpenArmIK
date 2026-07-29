/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace openarm::runtime {
namespace {

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
std::atomic<std::int64_t> fsync_failure_countdown{-1};
#endif

int persistence_fsync(int fd) {
#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
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
           std::strcmp(value, ".") != 0 && std::strcmp(value, "..") != 0 &&
           std::all_of(value, value + size, [](const char c) {
               return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
           });
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
    const int fd = openat(directory_fd, name, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
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

bool target_is_regular_or_absent(int directory_fd, const char *name, bool &exists) {
    struct stat status{};
    if (fstatat(directory_fd, name, &status, AT_SYMLINK_NOFOLLOW) == 0) {
        exists = true;
        return S_ISREG(status.st_mode) && !S_ISLNK(status.st_mode);
    }
    exists = false;
    return errno == ENOENT;
}

} // namespace
} // namespace openarm::runtime

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
extern "C" void oa_runtime_test_fail_fsync_after(std::int64_t countdown) {
    openarm::runtime::fsync_failure_countdown.store(countdown,
                                                    std::memory_order_relaxed);
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
        std::string checked_directory;
        if (!openarm::runtime::absolute_directory_path(absolute_directory,
                                                       checked_directory)) {
            return OA_RUNTIME_EPERMISSION;
        }
        auto authority = std::make_shared<openarm::runtime::PersistenceAuthorityData>();
        authority->directory_fd = open(checked_directory.c_str(),
                                       O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
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

extern "C" void oa_runtime_persistence_authority_destroy(
    oa_runtime_persistence_authority *authority) {
    openarm::runtime::persistence_authorities.erase(authority);
}

extern "C" oa_runtime_status oa_runtime_manifest_load_authenticated(
    const oa_runtime_persistence_authority *authority, const char *file_name,
    oa_runtime_manifest **out_manifest) {
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    const auto pinned = openarm::runtime::persistence_authorities.pin(authority);
    if (!pinned || !openarm::runtime::safe_file_name(file_name)) return OA_RUNTIME_EINVAL;
    try {
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        const oa_runtime_status status =
            openarm::runtime::load_authenticated_at(*pinned, file_name, manifest);
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
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    const auto authority_data = openarm::runtime::persistence_authorities.pin(authority);
    if (!manifest_data || !authority_data ||
        !openarm::runtime::safe_file_name(file_name)) {
        return OA_RUNTIME_EINVAL;
    }
    try {
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
            return OA_RUNTIME_EDURABILITY;
        }
        return OA_RUNTIME_OK;
    } catch (const std::bad_alloc &) {
        return OA_RUNTIME_ENOMEM;
    } catch (...) {
        return OA_RUNTIME_EIO;
    }
}
