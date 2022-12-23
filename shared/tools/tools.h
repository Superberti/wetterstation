//----------------------------------------------------------------------------
#ifndef TOOLS_H
#define TOOLS_H
//----------------------------------------------------------------------------
#include <exception>
#include <string>
#include <errno.h>
#include <vector>
#include <sstream>
#include <limits.h>
#include <cstdarg>
//----------------------------------------------------------------------------

/// String-Parsefunktion
/// \param line Zu parsende Zeile
/// \retval result Einzelne Bestandteile der Zeile
void ParseLine(const std::string& line, std::vector<std::string> & result);

/// Return and Endline aus einem C-String löschen
void KillReturnAndEndl(char * MyString);


// mal eben schnell eine Zahl in einen String umwandeln...
template<class T>std::string ToString(T Value){std::stringstream ss;ss<<Value;return ss.str();};

/// string -> double. Es wird erwartet, dass im String nur alphanumerische Zeichen und der Punkt
/// verwendet werden!
/// \param aValue Zahlenstring
/// \retval aNumber Extrahierter numerischer Wert
/// \return false: Umwandlung nicht erfolgreich
bool StringToDouble(const std::string aValue, double & aNumber);

/// string -> integer. Es wird erwartet, dass im String nur alphanumerische verwendet werden!
/// \param aValue Zahlenstring
/// \retval aNumber Extrahierter numerischer Wert
/// \return false: Umwandlung nicht erfolgreich
bool StringToInt(const std::string aValue, int & aNumber);

/// Funktionen zum Trimmen eines std::strings

/// Links trimmen
std::string& trimleft( std::string& s );
/// Rechts trimmen
std::string& trimright( std::string& s );
/// Beidseitig trimmen
std::string& trim( std::string& s );
/// Beidseitig trimmen, Original nicht verändern
std::string trim( const std::string& s );

/// Wandelt einen HEX-Codierten String in einen Binärpuffer um.
/// Die Puffergröße des Binärpuffers muss immer halb so groß sein, wie
/// der HEX-String. Die Größe des HEX-Strings muss immer gerade sein!
/// Also z.B. "41424344" -> "abcd"
/// \param aHex HEX-codierter String
/// \retval aBuffer Binärpuffer als Ergebnis der Kodierung
bool HexToBin(const std::string aHex, unsigned char * aBuffer);

/// Wandelt einen Binärpuffer in einen HEX-String um
/// Z.B. "abcd" -> "41424344"
/// \retval aHex Beinhaltet nach Kodierung den erzeugten HEX-String
/// \param aBuffer Binärpuffer
/// \param aBufferLength Länge des Binärpuffers
void BinToHex(std::string & aHex, const unsigned char * aBuffer, const unsigned int aBufferLength);

/// HexDump eines Speicherbereiches auf stdout ausgeben
/// \param Buffer Pointer auf Speicher
/// \param dwBytes Größe des Speicherbereiches
/// \param offset Offset zum Pointer
void HexDump(unsigned char * Buffer, const unsigned int dwBytes,const unsigned int offset=0);

/// In einen std::string mit printf drucken. Die Größe wird automatisch
/// angepasst. Oft sehr nützlich!
/// \retval aStr String, in den die Ausgabe erfolgen soll
/// \param format ... Format wie bei printf gewohnt und evtl. zusätzliche Parameter
/// \return Länge des Strings
int strprintf(std::string & aStr,const char* format, ...);

/// std::string mit printf erstellen. Die Größe wird automatisch angepasst. 
/// \param format ... Format wie bei printf gewohnt und evtl. zusätzliche Parameter
/// \return String mit printf-Ausgabe
std::string strprintf(const char* format, ...);

/// Hilfsfunktion für strprintf
int strvprintf(std::string & aStr,const char* format, va_list paramList);

/// Strings ersetzen
/// \param aString String, in dem etwas ersetzt werden soll
/// \param aToReplace Zeichen(kette), welche ersetzt werden soll
/// \param aReplaceWith Zeichen(kette), die als Ersatz dient
void ReplaceAll(std::string & aString, const std::string & aToReplace, const std::string & aReplaceWith);

/// Umwandlung Little- nach Big-Endian und umgekehrt
unsigned int SwapEndian(const unsigned int x);

/// CCITT-CRC16 berechnen
unsigned short compute_crc(unsigned char *data_ptr, unsigned short data_len);


#endif
