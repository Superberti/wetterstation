//----------------------------------------------------------------------------
#include <exception>
#include <iostream>
#include "tools.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
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
	return;
}

//----------------------------------------------------------------------------

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

		unsigned char offset = (aHex[i] >= 0x30 && aHex[i] <= 0x39) ? 0x30 : (aHex[i] >= 0x41 && aHex[i] <= 0x46) ? 0x37 : (aHex[i] >= 0x61 && aHex[i] <= 0x66) ? 0x57 : 0;
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

unsigned int SwapEndian(const unsigned int x)
{
	return ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) | ((x & 0x0000ff00) << 8) | ((x & 0x000000ff) << 24);
}

//----------------------------------------------------------------------------
