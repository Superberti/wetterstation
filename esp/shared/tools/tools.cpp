//----------------------------------------------------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <exception>
#include <iostream>
#include "tools.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "esp_log.h"
#include <regex.h>
//----------------------------------------------------------------------------

/// Parse-Funktion für Zeilen. Kommt auch mit
/// in Anführungszeichen stehenden Parametern
/// (also solche mit Whitespace) zurecht
void ParseLine(const std::string &line, std::vector<std::string> &result)
{
  result.clear();
  const char delim = ' ';
  std::string CurrentToken;
  char CurrentChar;
  const unsigned int LineSize = line.size();
  bool qm_on = false; // state quotation mark
  for (unsigned int i = 0; i < LineSize; i++)
  {
    CurrentChar = line[i];
    if (i == LineSize - 1)
    {
      if (CurrentChar != delim && CurrentChar != '"')
        CurrentToken += CurrentChar;
      if (CurrentToken.size())
      {
        result.push_back(CurrentToken);
        CurrentToken = "";
      }
    }
    if (CurrentChar == '"')
    {
      qm_on = !qm_on;
      continue;
    }
    if (qm_on)
    {
      CurrentToken += CurrentChar;
    }
    else if (CurrentChar == delim)
    {
      if (CurrentToken.size())
      {
        result.push_back(CurrentToken);
        CurrentToken = "";
      }
    }
    else
      CurrentToken += CurrentChar;
  }
}

void KillReturnAndEndl(char *MyString)
{
  const int s = strlen(MyString);
  for (int i = 0; i < s; i++)
  {
    if (MyString[i] == '\n' || MyString[i] == '\r')
    {
      MyString[i] = 0;
      break;
    }
  }
}

// -----------------------------------------------------------

std::vector<std::string> split(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    if (token.size())
      res.push_back(token);
  }
  token = s.substr(pos_start);
  if (token.size())
    res.push_back(s.substr(pos_start));
  return res;
}

// -----------------------------------------------------------

std::map<std::string, std::string> ParseHTTPPost(std::string aPOSTData)
{
  std::map<std::string, std::string> KVMap;
  std::vector<std::string> KeyValueLines = split(aPOSTData, "&");
  for (int i = 0; i < KeyValueLines.size(); i++)
  {
    std::vector<std::string> Elem = split(KeyValueLines[i], "=");
    if (Elem.size() == 2)
      KVMap.insert({Elem[0], Elem[1]});
  }
  return KVMap;
}

// -----------------------------------------------------------

bool GetIntFromStringMap(std::map<std::string, std::string> aMap, const std::string &aKey, int &aValue)
{
  std::map<std::string, std::string>::const_iterator pos = aMap.find(aKey);
  return pos == aMap.end() ? false : StringToInt(pos->second, aValue);
}

// -----------------------------------------------------------

bool GetValueFromStringMap(std::map<std::string, std::string> aMap, const std::string &aKey, std::string &aValue)
{
  std::map<std::string, std::string>::const_iterator pos = aMap.find(aKey);
  if (pos == aMap.end())
    return false;

  aValue = pos->second;
  return true;
}

// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------

bool StringToDouble(const std::string aValue, double &aNumber)
{
  const char *start = aValue.c_str();
  char *end = NULL;
  aNumber = strtod(start, &end);
  // end soll jetzt auch auf das Ende des strings zeigen, sonst Fehler!
  return (end - start == (int)aValue.size());
}

//----------------------------------------------------------------------------

bool StringToInt(const std::string aValue, int &aNumber)
{
  const char *start = aValue.c_str();
  char *end = NULL;
  aNumber = strtol(start, &end, 10);
  // end soll jetzt auch auf das Ende des strings zeigen, sonst Fehler!
  return (end - start == (int)aValue.size());
}

//----------------------------------------------------------------------------

std::string &trimleft(std::string &s)
{
  std::string::iterator it;

  for (it = s.begin(); it != s.end(); it++)
    if (!isspace(*it))
      break;

  s.erase(s.begin(), it);
  return s;
}

//----------------------------------------------------------------------------

std::string &trimright(std::string &s)
{
  std::string::difference_type dt;
  std::string::reverse_iterator it;

  for (it = s.rbegin(); it != s.rend(); it++)
    if (!isspace(*it))
      break;

  dt = s.rend() - it;

  s.erase(s.begin() + dt, s.end());
  return s;
}

//----------------------------------------------------------------------------

std::string &trim(std::string &s)
{
  trimleft(s);
  trimright(s);
  return s;
}

//----------------------------------------------------------------------------

std::string trim(const std::string &s)
{
  std::string t = s;
  return trim(t);
}

//----------------------------------------------------------------------------

bool HexToBin(const std::string aHex, unsigned char *aBuffer)
{
  unsigned char HighNibble(0);
  unsigned char LowNibble(0);
  const unsigned int HexSize = aHex.size();
  for (unsigned int i = 0; i < HexSize; i++)
  {
    if (!((aHex[i] >= 0x30 && aHex[i] <= 0x39) || (aHex[i] >= 0x41 && aHex[i] <= 0x46) || (aHex[i] >= 0x61 && aHex[i] <= 0x66)))
      return false;
    unsigned char offset = (aHex[i] >= 0x30 && aHex[i] <= 0x39) ? 0x30 : (aHex[i] >= 0x41 && aHex[i] <= 0x46) ? 0x37
                                                                     : (aHex[i] >= 0x61 && aHex[i] <= 0x66)   ? 0x57
                                                                                                              : 0;
    if (i % 2) // ungerade
    {
      LowNibble = (unsigned char)(aHex[i]) - offset;
      if (i)
      {
        unsigned char Value = (HighNibble << 4) + LowNibble;
        aBuffer[i / 2] = Value;
      }
    }
    else
      HighNibble = (unsigned char)(aHex[i]) - offset;
  }
  return true;
}

//----------------------------------------------------------------------------

void BinToHex(std::string &aHex, const unsigned char *aBuffer, const unsigned int aBufferLength)
{
  aHex = "";
  char HexValue[8];
  for (unsigned int i = 0; i < aBufferLength; i++)
  {
    sprintf(HexValue, "%02X", aBuffer[i]);
    aHex += HexValue;
  }
}

//----------------------------------------------------------------------------

void HexDump(unsigned char *Buffer, const unsigned int dwBytes, const unsigned int offset)
{
  std::string Dump, AsciiDump;
  char cell[10];
  const unsigned int BytesPerLine = 16;
  for (unsigned int i = 0; i < dwBytes; i++)
  {
    sprintf(cell, "%02X ", Buffer[i]);
    Dump += std::string(cell);
    AsciiDump += Buffer[i] >= 0x20 ? char(Buffer[i]) : '.';
    if ((i && !((i + 1) % BytesPerLine)) || (i == dwBytes - 1))
    {
      if (i == dwBytes - 1 && dwBytes % BytesPerLine)
      {
        // Letzte, unvollständige Zeile
        // Spaces einfügen
        Dump += std::string(BytesPerLine * 3 - Dump.size(), ' ');
        printf("%04Xh:%s | %s\n", i / BytesPerLine * BytesPerLine + offset, Dump.c_str(), AsciiDump.c_str());
      }
      else
      {
        // Zeile rausschreiben
        printf("%04Xh:%s | %s\n", (i - 1) / BytesPerLine * BytesPerLine + offset, Dump.c_str(), AsciiDump.c_str());
        Dump = "";
        AsciiDump = "";
      }
    }
  }
}

//----------------------------------------------------------------------------

int64_t GetTime_us()
{
  return xTaskGetTickCount()*1000;//esp_timer_get_time();
}

//----------------------------------------------------------------------------

int strprintf(std::string &aStr, const char *format, ...)
{
  int rc;
  va_list paramList;
  va_start(paramList, format);
  rc = strvprintf(aStr, format, paramList);
  va_end(paramList);
  return rc;
}

//----------------------------------------------------------------------------

std::string strprintf(const char *format, ...)
{
  std::string text;
  va_list paramList;
  va_start(paramList, format);
  strvprintf(text, format, paramList);
  va_end(paramList);
  return text;
}

//----------------------------------------------------------------------------

int strvprintf(std::string &aStr, const char *format, va_list paramList)
{
  va_list copy;
  va_copy(copy, paramList);
  int size = vsnprintf(NULL, 0, format, copy);
  va_end(copy);
  if (!size)
  {
    aStr = "";
    return 0;
  }
  char *ar = new char[size + 2];
  va_copy(copy, paramList);
  // Unter gcc scheint size eins zu klein zu sein. Im Borland-Compiler
  // ist alles OK
  size = vsnprintf(ar, size + 1, format, paramList);
  va_end(copy);
  aStr = ar;
  delete[] ar;
  return size;
}

//----------------------------------------------------------------------------

void ReplaceAll(std::string &aString,
                const std::string &aToReplace,
                const std::string &aReplaceWith)
{
  if ((aToReplace.size() == 0) || (aToReplace == aReplaceWith))
    return;
  unsigned int iPos = 0;
  while ((iPos = aString.find(aToReplace, iPos)) != std::string::npos)
  {
    aString.replace(iPos, aToReplace.size(), aReplaceWith);
    iPos += aReplaceWith.size();
  }
}

//----------------------------------------------------------------------------

timespec ts_diff(const timespec &start, const timespec &end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

//----------------------------------------------------------------------------

void ts_add(timespec &MyTime, const timespec &MyAddend)
{
  if ((MyTime.tv_nsec + MyAddend.tv_nsec) >= 1000000000)
  {
    MyTime.tv_sec += MyAddend.tv_sec + 1;
    MyTime.tv_nsec = MyTime.tv_nsec + MyAddend.tv_nsec - 1000000000;
  }
  else
  {
    MyTime.tv_sec += MyAddend.tv_sec;
    MyTime.tv_nsec += MyAddend.tv_nsec;
  }
}

//----------------------------------------------------------------------------

unsigned int SwapEndian(const unsigned int x)
{
  return ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) | ((x & 0x0000ff00) << 8) | ((x & 0x000000ff) << 24);
}

//----------------------------------------------------------------------------

/// Boyer-Moore-Suchklasse
TBMFinder::TBMFinder()
    : mArrayToFind(NULL), IsCopy(false)
{
  InitOK = false;
  len = 0;
}

TBMFinder::~TBMFinder()
{
  if (mArrayToFind && IsCopy)
    delete[] mArrayToFind;
}

//----------------------------------------------------------------------------

void TBMFinder::Init(const unsigned char *const ArrayToFind, const unsigned int ArrayLen, const bool copy)
{
  size_t i;
  len = ArrayLen;
  for (i = 0; i <= UCHAR_MAX; ++i)
    table[i] = len;
  for (i = 0; i < len; ++i)
    table[ArrayToFind[i]] = len - i - 1;
  // Kopie erzeugen
  if (mArrayToFind && IsCopy)
    delete[] mArrayToFind;
  if (copy)
  {
    mArrayToFind = new unsigned char[ArrayLen];
    memcpy(mArrayToFind, ArrayToFind, ArrayLen);
  }
  else
    mArrayToFind = const_cast<unsigned char *>(ArrayToFind);

  IsCopy = copy;
  InitOK = true;
}

//----------------------------------------------------------------------------

unsigned int TBMFinder::Find(const unsigned char *const Source, const unsigned int limit)
{
  if (!InitOK)
    return 0;

  size_t shift = 0;
  size_t pos = len - 1;

  while (pos < limit)
  {
    while (pos < limit && (shift = table[Source[pos]]) > 0)
    {
      pos += shift;
    }
    if (0 == shift)
    {
      if (0 == memcmp(mArrayToFind, Source + pos - len + 1, len))
      {
        return (pos - len + 1);
      }
      else
        ++pos;
    }
  }
  return 0xffffffff;
}

//----------------------------------------------------------------------------

/// Suche mit impliziter Initialisierung
unsigned int TBMFinder::Find(const unsigned char *const Source, const unsigned int limit,
                             const unsigned char *const ArrayToFind, const unsigned int ArrayLen)
{
  // Array nicht kopieren, da ArrayToFind für die Laufzeit der Funktion immer
  // gültig ist
  Init(ArrayToFind, ArrayLen, false);
  return Find(Source, limit);
}

//----------------------------------------------------------------------------

unsigned short compute_crc(unsigned char *data_ptr, unsigned short data_len)
{
  unsigned short x16;
  int i;
  unsigned short crc_buf(0);
  for (unsigned short it = 0; it < data_len; it++)
  {
    unsigned char input = data_ptr[it];
    for (i = 0; i < 8; i++)
    {
      if ((crc_buf & 0x0001) ^ (input & 0x01))
        x16 = 0x8408;
      else
        x16 = 0x0000;
      crc_buf = crc_buf >> 1;
      crc_buf ^= x16;
      input = input >> 1;
    }
  }
  return crc_buf;
}

//----------------------------------------------------------------------------

std::string clean_path(const std::string &path)
{
  std::string new_path;
  char sep = '/';
  for (auto i = 0; i < path.size(); ++i)
  {
    if ((path[i] == sep && !new_path.empty() && new_path.back() == sep) || path[i] == '?')
      continue;
    new_path.push_back(path[i]);
  }
  return new_path;
}

//----------------------------------------------------------------------------

std::string urlencode(const std::string &str)
{
  std::string encodedString = "";
  char c;
  char code0;
  char code1;
  for (int i = 0; i < str.length(); i++)
  {
    c = str[i];
    if (c == ' ')
    {
      encodedString += '+';
    }
    else if (isalnum(c))
    {
      encodedString += c;
    }
    else
    {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9)
      {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9)
      {
        code0 = c - 10 + 'A';
      }
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
    }
  }
  return encodedString;
}

//----------------------------------------------------------------------------

std::string urldecode(const std::string &str)
{
  std::string encodedString = "";
  char c;
  char code0;
  char code1;
  for (int i = 0; i < str.length(); i++)
  {
    c = str[i];
    if (c == '+')
    {
      encodedString += ' ';
    }
    else if (c == '%')
    {
      i++;
      code0 = str[i];
      i++;
      code1 = str[i];
      c = (h2int(code0) << 4) | h2int(code1);
      encodedString += c;
    }
    else
    {

      encodedString += c;
    }
  }

  return encodedString;
}

//----------------------------------------------------------------------------

unsigned char h2int(char c)
{
  if (c >= '0' && c <= '9')
  {
    return ((unsigned char)c - '0');
  }
  if (c >= 'a' && c <= 'f')
  {
    return ((unsigned char)c - 'a' + 10);
  }
  if (c >= 'A' && c <= 'F')
  {
    return ((unsigned char)c - 'A' + 10);
  }
  return (0);
}

//----------------------------------------------------------------------------

bool IsEmptyOrWhiteSpaces(const std::string &aText)
{
  return aText.empty() || std::all_of(aText.begin(), aText.end(), isspace);
}

//----------------------------------------------------------------------------

LogBuffer::LogBuffer(uint16_t aMaxLines, uint16_t aLineSize)
{
  mCurrentLineNumber = 0;
  mCurrentNumOfLines = 0;
  mMaxLines = aMaxLines;
  mLineSize = aLineSize;
  mLineBuffer = new char[aMaxLines * aLineSize];
  memset(mLineBuffer, 0, aMaxLines * aLineSize);
  if (mLineBuffer == NULL)
  {
    ESP_LOGE("LogBuffer", "Could not create log buffer...");
  }
}

LogBuffer::~LogBuffer()
{
  delete[] mLineBuffer;
}

void LogBuffer::AddLine(const std::string &aLine)
{
  if (mLineBuffer == NULL || IsEmptyOrWhiteSpaces(aLine))
    return;
  
  char *LineStart = mLineBuffer + mCurrentLineNumber * mLineSize;
  memset(LineStart, 0, mLineSize);
  int NumCopied = strlcpy(LineStart, aLine.c_str(), mLineSize);
  // Zeile auf jeden Fall umbrechen
  bool LineBreaked = aLine[aLine.length() - 1] == '\n';
  if (!LineBreaked)
  {
    int LFColumn = std::min(NumCopied, mLineSize - 2);
    LineStart[LFColumn] = '\n';
    LineStart[LFColumn + 1] = 0;
  }
  mCurrentLineNumber++;
  mCurrentLineNumber %= mMaxLines;
  mCurrentNumOfLines++;
  if (mCurrentNumOfLines > mMaxLines)
    mCurrentNumOfLines = mMaxLines;
}

// Liest eine Zeile aus dem Logpuffer. 0=Zuletzt geschriebene Zeile ... mCurrentNumOfLines-1
bool LogBuffer::GetHistLine(char *aLineBuf, uint16_t aBufSize, uint16_t aHistLineNumber)
{
  if (aHistLineNumber >= mCurrentNumOfLines)
    return false;
  int BufLineNumber = (int)(mCurrentLineNumber - 1) - aHistLineNumber;
  if (BufLineNumber < 0)
    BufLineNumber = mMaxLines + BufLineNumber;
  strlcpy(aLineBuf, mLineBuffer + BufLineNumber * mLineSize, aBufSize);
  return true;
}

int regex_replace(std::string &aText, const std::string &aPattern, const std::string &aReplaceText)
{
  char *str1 = (char *)malloc(aText.length() + 1);
  strlcpy(str1, aText.c_str(), aText.length() + 1);
  int err = regex_replace(&str1, aPattern.c_str(), aReplaceText.c_str());
  aText = str1;
  free(str1);
  return err;
}

int regex_replace(char **str, const char *pattern, const char *replace)
{
  // replaces regex in pattern with replacement observing capture groups
  // *str MUST be free-able, i.e. obtained by strdup, malloc, ...
  // back references are indicated by char codes 1-31 and none of those chars can be used in the replacement string such as a tab.
  // will not search for matches within replaced text, this will begin searching for the next match after the end of prev match
  // returns:
  //   -1 if pattern cannot be compiled
  //   -2 if count of back references and capture groups don't match
  //   otherwise returns number of matches that were found and replaced
  //
  regex_t reg;
  unsigned int replacements = 0;
  // if regex can't commpile pattern, do nothing
  if (!regcomp(&reg, pattern, REG_EXTENDED))
  {
    size_t nmatch = reg.re_nsub;
    regmatch_t m[nmatch + 1];
    const char *rpl, *p;
    // count back references in replace
    int br = 0;
    p = replace;
    while (1)
    {
      while (*++p > 31)
        ;
      if (*p)
        br++;
      else
        break;
    } // if br is not equal to nmatch, leave
    if (br != nmatch)
    {
      regfree(&reg);
      return -2;
    }
    // look for matches and replace
    char *NewString;
    char *search_start = *str;
    while (!regexec(&reg, search_start, nmatch + 1, m, REG_NOTBOL))
    {
      // make enough room
      NewString = (char *)malloc(strlen(*str) + strlen(replace));
      if (!NewString)
        exit(EXIT_FAILURE);
      *NewString = '\0';
      strncat(NewString, *str, search_start - *str);
      p = rpl = replace;
      int c;
      strncat(NewString, search_start, m[0].rm_so); // test before pattern
      for (int k = 0; k < nmatch; k++)
      {
        while (*++p > 31)
          ;                               // skip printable char
        c = *p;                           // back reference (e.g. \1, \2, ...)
        strncat(NewString, rpl, p - rpl); // add head of rpl
        // concat match
        strncat(NewString, search_start + m[c].rm_so, m[c].rm_eo - m[c].rm_so);
        rpl = p++; // skip back reference, next match
      }
      strcat(NewString, p); // trailing of rpl
      unsigned int new_start_offset = strlen(NewString);
      strcat(NewString, search_start + m[0].rm_eo); // trailing text in *str
      free(*str);
      *str = (char *)malloc(strlen(NewString) + 1);
      strcpy(*str, NewString);
      search_start = *str + new_start_offset;
      free(NewString);
      replacements++;
    }
    regfree(&reg);
    // ajust size
    *str = (char *)realloc(*str, strlen(*str) + 1);
    return replacements;
  }
  else
  {
    return -1;
  }
}

/// CCITT-CRC16 berechnen
unsigned short Crc16(unsigned char *data_ptr, unsigned short data_len)
{
	unsigned short x16;
	int i;
	unsigned short crc_buf(0);
	for (unsigned short it = 0; it < data_len; it++)
	{
		unsigned char input = data_ptr[it];
		for (i = 0; i < 8; i++)
		{
			if ((crc_buf & 0x0001) ^ (input & 0x01))
				x16 = 0x8408;
			else
				x16 = 0x0000;
			crc_buf = crc_buf >> 1;
			crc_buf ^= x16;
			input = input >> 1;
		}
	}
	return crc_buf;
}