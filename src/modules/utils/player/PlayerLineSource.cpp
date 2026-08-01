#include "PlayerLineSource.h"

#include "libs/FirmwareFileSystem.h"

PlayerLineSource::ReadResult FilePlayerLineSource::read(char *buffer, std::size_t size)
{
    if (file_ == nullptr || buffer == nullptr || size == 0) {
        return ReadResult::error;
    }
    if (fwfs::fgets(buffer, size, file_) != nullptr) {
        return ReadResult::data;
    }
    return fwfs::feof(file_) ? ReadResult::end : ReadResult::error;
}

bool FilePlayerLineSource::at_end() const
{
    return file_ != nullptr && fwfs::feof(file_);
}

unsigned long FilePlayerLineSource::position() const
{
    if (file_ == nullptr) return 0;
    const long offset = fwfs::ftell(file_);
    return offset > 0 ? static_cast<unsigned long>(offset) : 0;
}

void FilePlayerLineSource::close()
{
    if (file_ != nullptr) {
        fwfs::fclose(file_);
        file_ = nullptr;
    }
}
