//----------------------------------------------------------------------------
#ifndef TOOLS_H
#define TOOLS_H
//----------------------------------------------------------------------------
#include <string>
#include <errno.h>
#include <vector>
#include <sstream>
#include <limits.h>
#include <iostream>
#include <map>
#include <list>
//----------------------------------------------------------------------------

/// String-Parsefunktion
/// \param line Zu parsende Zeile
/// \retval result Einzelne Bestandteile der Zeile
void ParseLine(const std::string& line, std::vector<std::string> & result);

/// Return and Endline aus einem C-String löschen
void KillReturnAndEndl(char * MyString);

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

/// Aktuelle Zeit in Mikrosekunden
int64_t GetTime_us();

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

/// Differenz von timespec-Strukturen bilden. Liefert end-start (Minuend-Subtrahent)
/// \param start Subtrahend
/// \param end Minuend
/// \return Differenz
timespec ts_diff(const timespec & start, const timespec & end);

/// Summe von zwei timespec-Strukturen bilden (MyTime+=MyAddend
/// \retval MyTime Wert der Summe
/// \param MyAddend Summand
void ts_add(timespec & MyTime, const timespec & MyAddend);

/// Umwandlung Little- nach Big-Endian und umgekehrt
unsigned int SwapEndian(const unsigned int x);

// string aufsplitten nach Trennzeichen
std::vector<std::string> split(std::string s, std::string delimiter);

// Erstellt eine Key/Value map aus einer HTTP-POST-Antwort (application/x-www-form-urlencoded)
// z.B. "year=2008&month=1&day=23&hour=3&minute=23&second=50"
std::map<std::string, std::string> ParseHTTPPost(std::string aPOSTData);

// Liest einen String aus einer Map ein und versucht ihn in einen Integer umzuwandeln
bool GetIntFromStringMap(std::map<std::string, std::string> aMap, const std::string & aKey, int & aValue);

// String-Wert aus einer String-Map lesen
bool GetValueFromStringMap(std::map<std::string, std::string> aMap, const std::string & aKey, std::string & aValue);

// Löscht doppelte Slashes aus einem Pfad
std::string clean_path(const std::string &path);

/// Klasse zur Implementation des Boyer-Moore-Suchalgorithmus. Vor Benutzung muss
/// mit Init() die Suchtabelle initialisiert werden. Es gibt auch eine Funktion,
/// die implizit Init() vor jeder Suchanfrage auslöst. Bei großen Arrays füllt
/// dieser Overhead dann kaum noch ins Gewicht. Bei Arrays
/// um ein Megabyte ist Boyer-Moore ca. 10 mal schneller. Bei sehr kleinen Arrays und
/// ständig wechselnden Suchwörtern wird der Vorteil geringer.
class TBMFinder
{
	protected:
  	/// Sprungtabelle
		unsigned int table[UCHAR_MAX + 1];
  	/// Länge des Suchworts
		unsigned int len;
  	/// Pointer auf Suchbereich
		unsigned char * mArrayToFind;
  	/// Merker, ob bereits initialisiert
		bool InitOK;
		/// Merker, ob ArrayToFind kopiert wurde oder nicht.
		bool IsCopy;
	public:
  	/// CTOR
		TBMFinder();
		~TBMFinder();
  	/// Initialisierung der Suchtabelle
  	/// \param ArrayToFind Pointer auf das zu suchende Muster
  	/// \param ArrayLen Länge des Suchmusters
		/// \param copy ArrayToFind wird kopiert, ansonsten wird sich nur der Pointer gemerkt.
		void Init(const unsigned char * const ArrayToFind, const unsigned int ArrayLen, const bool copy=true);
  	/// Findet (oder auch nicht) ein Suchmuster in einem Speicherblock.
  	/// Ist nicht auf C-Strings beschränkt (Findet also auch Nullen)
  	/// Exception, falls vorher nicht Initialisiert wurde!
  	/// \param Source Speicherblock, in dem gesucht werden soll
  	/// \param limit Suchraumgröße in bytes
  	/// \return Erste gefundene Suchposition, MAXDWORD falls nicht gefunden.
		unsigned int Find( const unsigned char * const Source, const unsigned int limit);
  	/// Suche mit impliziter Initialisierung
		unsigned int Find( const unsigned char * const Source, const unsigned int limit,
											 const unsigned char * const ArrayToFind, const unsigned int ArrayLen);
};

unsigned char h2int(char c);
std::string urldecode(const std::string &str);
std::string urlencode(const std::string &str);
bool IsEmptyOrWhiteSpaces(const std::string & aText);

// Puffer für die x letzten Zeilen des Logs. Mit Absicht ziemlich statisch gehalten, da so
// keine Speicherfragmentierung auf Dauer zu befürchten ist.
class LogBuffer
{
  protected:
    uint16_t mMaxLines;
    uint16_t mLineSize;
    uint16_t mCurrentLineNumber;
    uint16_t mCurrentNumOfLines;
    
    char* mLineBuffer;
  public:
    LogBuffer(uint16_t aMaxLines, uint16_t aLineSize);
    ~LogBuffer();
    void AddLine(const std::string &aLine);
    bool GetHistLine(char* aLineBuf,uint16_t aBufSize, uint16_t aHistLineNumber);
    uint16_t GetNumOfStoredLines(){return mCurrentNumOfLines;}
    uint16_t GetLineSize(){return mLineSize;}
    char* GetLineBuffer(){return mLineBuffer;}
    uint16_t GetLineBufSize(){return mMaxLines*mLineSize;}
};

int regex_replace(std::string & aText, const std::string & aPattern, const std::string & aReplaceText);
int regex_replace(char **str, const char *pattern, const char *replace);

/// CCITT-CRC16 berechnen
unsigned short Crc16(unsigned char *data_ptr, unsigned short data_len);

#endif
