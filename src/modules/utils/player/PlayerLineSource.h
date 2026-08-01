#ifndef PLAYERLINESOURCE_H
#define PLAYERLINESOURCE_H

#include <cstddef>
#include <cstdio>

class PlayerLineSource {
public:
    enum class ReadResult {
        data,
        waiting,
        end,
        error
    };

    virtual ~PlayerLineSource() = default;
    virtual ReadResult read(char *buffer, std::size_t size) = 0;
    virtual bool at_end() const = 0;
    virtual unsigned long position() const = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
};

class FilePlayerLineSource final : public PlayerLineSource {
public:
    void attach(FILE *file) { file_ = file; }
    FILE *file() const { return file_; }

    ReadResult read(char *buffer, std::size_t size) override;
    bool at_end() const override;
    unsigned long position() const override;
    void close() override;
    bool is_open() const override { return file_ != nullptr; }

private:
    FILE *file_ = nullptr;
};

#endif
