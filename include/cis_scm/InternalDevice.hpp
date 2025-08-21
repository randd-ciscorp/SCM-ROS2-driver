#ifndef INTERNALDEVICE_HPP
#define INTERNALDEVICE_HPP

#include <sys/ioctl.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <string>

#include "Device.h"
#include "tof.h"
#include "scmcap.h"
#define N_CAP_BUF	4

// Error type
#define S_OK			0
#define E_POINTER		EFAULT
#define E_FAIL			ENODATA
#define E_INVALIDARG	EINVAL
#define E_OUTOFMEMORY	ENOMEM
#define E_HANDLE		ENODATA

namespace cis_scm{

namespace internal{

class InternalDevice : public Device
{
protected:
    InternalDevice();
    ~InternalDevice();

public:
    virtual int connect() = 0;
    virtual void disconnect() = 0;

    DevInfo getInfo() const;
	int getData(uint8_t* data) override = 0;

protected:
    DevInfo devInfo_;
    cis::CameraEvent camEvent_;

    unsigned int bufInd_;
	struct RequestBuffer* buffers_;
};

class ToFInternalDevice : public InternalDevice
{
public:
    ToFInternalDevice(){};
    ToFInternalDevice(const std::string &path);
    ~ToFInternalDevice();

    int connect() override;
    void disconnect() override;

    int getData(uint8_t *data) override;

    cis::CalibData getCalibData() { return calibData_; };
    cis::ToFParam getToFParams() { return tofParams_; };

private:
    cis::CameraIMX570 camInputDevice_;
    cis::CalibData calibData_;
    cis::ToFParam tofParams_;

};

class RGBInternalDevice : public InternalDevice
{
public:
    RGBInternalDevice(){};
    RGBInternalDevice(cis::CameraAR0234* camInputDevice);
    RGBInternalDevice(cis::CameraIMX715* camInputDevice);
    ~RGBInternalDevice();

    int connect() override;
    void disconnect() override;

    int getData(uint8_t *data) override;

private:
    cis::CameraInputColor* camInputDevice_;
};

class RGBDInternalDevice : public InternalDevice
{
public:
    RGBDInternalDevice(const std::string &path, bool align);

    int connect() override;
    void disconnect() override;

    int getData(uint8_t *data) override;

private:
    ToFInternalDevice tofIntDev;
    RGBInternalDevice rgbIntDev;

    bool isAligned;
};
}
}

#endif
