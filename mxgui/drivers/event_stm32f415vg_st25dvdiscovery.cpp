
/***************************************************************************
 *   Copyright (C) 2014 by Terraneo Federico and Roberto Morlacchi and     *
 *   Domenico Frasca'                                                      *
 *   Copyright (C) 2024 by Daniele Cattaneo                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *    
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include <config/mxgui_settings.h>

#if defined(_BOARD_STM32F415VG_ST25DVDISCOVERY) && defined(MXGUI_LEVEL_2)

#include "event_stm32f415vg_st25dvdiscovery.h"
#include "miosix.h"
#include "kernel/scheduler/scheduler.h"
#include "util/software_i2c.h"
#include <algorithm>

using namespace std;
using namespace miosix;

static Semaphore touchIntSema;

/**
 * Touchscreen interrupt
 */
void __attribute__((naked)) EXTI9_5_IRQHandler()
{
    saveContext();
    asm volatile("bl EXTI9_5HandlerImpl");
    restoreContext();
}

/**
 * Touchscreen interrupt actual implementation
 */
extern "C" void __attribute__((used)) EXTI9_5HandlerImpl()
{
    EXTI->PR = EXTI_PR_PR8;  // Assuming the touchscreen interrupt is on EXTI line 8
    touchIntSema.IRQsignal();
}

namespace mxgui {

typedef Gpio<GPIOG_BASE, 6> buttonKey;
typedef Gpio<GPIOB_BASE, 12> buttonTamper;
typedef Gpio<GPIOA_BASE, 0> buttonWakeup;
typedef Gpio<GPIOB_BASE, 8> scl;
typedef Gpio<GPIOB_BASE, 9> sda;
typedef Gpio<GPIOI_BASE, 8> interrupt;

typedef SoftwareI2C<sda, scl> ioExtI2C;

/**
 * The registers of the stmpe811 touchscreen controller
 */
enum stmpe811regs
{
    SYS_CTRL1 = 0x03,
    SYS_CTRL2 = 0x04,
    INT_CTRL = 0x09,
    GPIO_SET_PIN = 0x10,
    GPIO_CLR_PIN = 0x11,
    GPIO_MP_STA = 0x12,
    GPIO_DIR = 0x13,
    GPIO_ALT_FUNC = 0x17,
    INT_EN = 0x0a,
    INT_STA = 0x0B,
    TSC_CTRL = 0x40,
    TSC_CFG = 0x41,
    FIFO_TH = 0x4a,
    FIFO_STA = 0x4b,
    TSC_DATA_XYZ = 0xd7,
    FIFO_SIZE = 0x4C
};

template <class I2C, int Addr>
class STMPE811
{
public:
    /**
     * Write into a register in the stmpe811
     * \param reg register number
     * \param val value to be written in the register
     */
    void writeReg(unsigned char reg, unsigned char val)
    {
        I2C::sendStart();
        I2C::send(Addr);
        I2C::send(reg);
        I2C::send(val);
        I2C::sendStop();
    }

    /**
     * Read from a register of the stmpe811
     * \param reg register number
     * \param n number of bytes to read from register
     * \param pointer to a memory area of at least n bytes where the read data will
     * be stored
     */
    void readReg(unsigned char reg, int n, unsigned char *result)
    {
        if (n <= 0) return;
        I2C::sendStart();
        I2C::send(Addr);
        I2C::send(reg);
        I2C::sendStop();
        I2C::sendStart();
        I2C::send(Addr | 1);
        for (int i = 0; i < n - 1; i++) result[i] = I2C::recvWithAck();
        result[n - 1] = I2C::recvWithNack();
        I2C::sendStop();
    }

    /**
     * Perform initial configuration of the chip.
     */
    void init(void)
    {
        // To let the I2C voltages settle
        Thread::sleep(5);

        writeReg(SYS_CTRL1, 0x02); // SOFT_RESET=1
        Thread::sleep(10);
        writeReg(SYS_CTRL1, 0x00); // SOFT_RESET=0
        Thread::sleep(2);
        writeReg(SYS_CTRL2, 0x08); // !GPIO_OFF !TSC_OFF !ADC_OFF
    }

    /**
     * Clear the stmpe811 fifo
     */
    void touchFifoClear()
    {
        writeReg(FIFO_STA, 0x01); // RESET FIFO
        writeReg(FIFO_STA, 0x00); // RESET FIFO
    }

    /**
     * Configure the chip as a resistive touchscreen controller.
     */
    void initTouch()
    {
        // Total time to read the touchscreen is
        // TDD*2+SETTLING*3+AVE*17.2us*3= ~ 17.5ms
        writeReg(TSC_CFG, 0xe4); // TSC_CFG= AVE=8, TDD=1ms, SETTLING=5ms
        writeReg(FIFO_TH, 0x01); // FIFO_TH= 1
        touchFifoClear();

        // This may allow the chip to go out of hibernate once touch detected
        writeReg(INT_CTRL, 0x01);
        writeReg(INT_EN, 0x03);

        // TSC_CTRL values:
        // 1     bit 0     enabled: yes
        // 001   bit 1:3   TSC mode: X, Y only
        // 011   bit 4:6   tracking index (minimum ADC delta for triggering move): 16
        // 0     bit 7     TSC status (read only)
        writeReg(TSC_CTRL, 0b0'011'001'1);
        writeReg(FIFO_TH, 0x01);
    }

    /**
     * \return the touch point or (-1,-1) if no touch is in progress
     */
    Point getTouchData()
    {
        unsigned char ctrl;
        // Check if touch detected by polling the CTRL register
        readReg(TSC_CTRL, 1, &ctrl);
        if ((ctrl & 0x80) == 0)
        {
            // No touch
            lastTouchPoint = Point(-1, -1);
            return lastTouchPoint;
        }
        else
        {
            // Touch detected, check if there are samples in FIFO.
            // Even if a touch is detected, the FIFO may be empty if:
            // - the first/next sample is not yet ready (for example because of
            //   settling time)
            // - the pen is standing still and window tracking has discarded
            //   some samples
            // In this case, reading from TSC_DATA_XYZ will return all zeros.
            // To avoid returning incorrect event coordinates we check the FIFO
            // level, and if it is zero we simply return the last point again.
            unsigned char fifoFillLevel;
            readReg(FIFO_SIZE, 1, &fifoFillLevel);
            if (fifoFillLevel == 0) return lastTouchPoint;

            // Read the new sample
            unsigned char tsData[3];
            readReg(TSC_DATA_XYZ, 3, tsData);
            touchFifoClear();
            int x = static_cast<int>(tsData[0]) << 4 | tsData[1] >> 4;
            int y = ((static_cast<int>(tsData[1]) & 0xf) << 8) | tsData[2];
            y = 4095 - y; // Y is swapped

            // Apply calibration. Values may vary from unit to unit
            const int xMin = 220;
            const int xMax = 3900;
            const int yMin = 160;
            const int yMax = 3900;
            x = (x - xMin) * 240 / (xMax - xMin);
            y = (y - yMin) * 320 / (yMax - yMin);

            lastTouchPoint = Point(x, y);
            return lastTouchPoint;
        }
    }

private:
    Point lastTouchPoint = Point(-1, -1);
};

//
// class InputHandlerImpl
//

class InputHandlerImpl : public InputHandler
{
public:
    InputHandlerImpl();

    /**
     * \return the currently pressed key
     */
    InputEvent getEvent();
};

InputHandlerImpl::InputHandlerImpl()
{
    buttonKey::mode(Mode::INPUT);
    buttonTamper::mode(Mode::INPUT);
    buttonWakeup::mode(Mode::INPUT);
    scl::mode(Mode::ALTERNATE);
    scl::alternateFunction(4);
    sda::mode(Mode::ALTERNATE);
    sda::alternateFunction(4);
    interrupt::mode(Mode::INPUT);

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI8_PI;
    EXTI->IMR |= EXTI_IMR_MR8;
    EXTI->EMR |= EXTI_EMR_MR8;
    EXTI->RTSR |= EXTI_RTSR_TR8;
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // Disable I2C to avoid problems with stmpe811 initialization
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    // Enable clock to I2C
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Initialize the touchscreen controller
    static STMPE811<ioExtI2C, 0x82> touchscreen;
    touchscreen.init();
    touchscreen.initTouch();
}

/**
 * Polls the state of the keyboard
 */
InputEvent InputHandlerImpl::getEvent()
{
    // Map of buttons
    if (buttonKey::value() == 0) return InputEvent::KeyboardEvent('A', true);
    if (buttonTamper::value() == 0) return InputEvent::KeyboardEvent('B', true);
    if (buttonWakeup::value() == 0) return InputEvent::KeyboardEvent('C', true);

    // Map of the touchscreen
    touchIntSema.wait();
    auto touch = touchscreen.getTouchData();
    if (touch == Point(-1, -1)) return InputEvent();
    return InputEvent::TouchEvent(touch);
}

} // namespace mxgui

#endif // _BOARD_STM32F415VG_ST25DVDISCOVERY
