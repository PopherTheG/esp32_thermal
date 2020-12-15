#ifndef COMPONENTS_SCANNER_HW1258
#define COMPONENTS_SCANNER_HW1258

#include "scanner_app.h"

#define ACK             0x06
#define NACK            0x15

#define SCAN            "^_^SCAN."
#define SLEEP           "^_^SLEEP."

#define SAVE            "^_^SETSAV."
#define QRCENA          "^_^QRCENA1."
#define SCMMDT          "^_^SCMMDT."
#define MDTMIT          "^_^MDTMIT1."
#define MDTSTA          "^_^MDTSTA2."
#define MDTGUN33        "^_^MDTGUN33."
#define MDTEXT1         "^_^MDTEXT1."
#define CNTALW0         "^_^CNTALW0."
#define BREENA0         "^_^BREENA0"

scanner_status_t hw1258_init(void);

scanner_status_t hw1258_get_expected_response(void);

#endif /* COMPONENTS_SCANNER_HW1258 */
