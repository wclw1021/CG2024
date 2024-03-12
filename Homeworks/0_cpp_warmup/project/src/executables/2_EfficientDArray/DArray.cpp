// implementation of class DArray
#include "DArray.h"
#include <iostream>
#include <cassert>
using namespace std;

// default constructor
DArray::DArray() {
	Init();
}

// set an array with default values
DArray::DArray(int nSize, double dValue) 
		: m_pData(new double[nSize]), m_nSize(nSize), m_nMax(nSize)  //initializing
{
	for (int i = 0; i < nSize; i++)
		m_pData[i] = dValue;
}

DArray::DArray(const DArray& arr) 
		: m_pData(new double[arr.m_nSize]), m_nSize(arr.m_nSize), m_nMax(arr.m_nSize)
{
	for (int i = 0; i < arr.m_nSize; i++)
		m_pData[i] = arr[i];
}

// deconstructor
DArray::~DArray() {
	Free();
}

// display the elements of the array
void DArray::Print() const {
	cout << "size = " << m_nSize << ":";
	for (int i = 0; i < m_nSize; i++)
		cout << " " << GetAt(i);
	cout << endl;
}

// initilize the array
void DArray::Init() {
	m_nSize = 0;
	m_pData = nullptr;
	m_nMax = 0;
}

// free the array
void DArray::Free() {
	delete[] m_pData;
	m_pData = nullptr;
	m_nSize = 0;
	m_nMax = 0;
}

// get the size of the array
int DArray::GetSize() const {
	return m_nSize; 
}

// allocate enough memory
void DArray::Reserve(int nSize) {
	if (m_nMax >= nSize)
		return;
	while (m_nMax < nSize)
		m_nMax = m_nMax == 0 ? 1 : 2 * m_nMax;

	double* pData = new double[m_nMax];
	// memcpy func: void* memcpy(void* destination, const void* source, size_t num);
	memcpy(pData, m_pData, m_nSize * sizeof(double));
	delete[] m_pData;
	m_pData = pData;
}

// set the size of the array
void DArray::SetSize(int nSize) {
	if (nSize == m_nSize)
	return;
	Reserve(nSize);
	for (int i = m_nSize; i < nSize; i++)
		m_pData[i] = 0;
	m_nSize = nSize;
}

// get an element at an index
const double& DArray::GetAt(int nIndex) const {
	assert(nIndex >= 0 && nIndex < m_nSize);
	return m_pData[nIndex]; // you should return a correct value
}

// set the value of an element 
void DArray::SetAt(int nIndex, double dValue) {
	assert(nIndex >= 0 && nIndex < m_nSize);
	m_pData[nIndex] = dValue;
}

// whether changeable according to reference or not
// overload operator '[]'
double& DArray::operator[](int nIndex) {
	assert(nIndex >= 0 && nIndex < m_nSize);
	return m_pData[nIndex];
}

// overload operator '[]'
const double& DArray::operator[](int nIndex) const {
	assert(nIndex >= 0 && nIndex < m_nSize);
	return m_pData[nIndex];
}

// add a new element at the end of the array
void DArray::PushBack(double dValue) {
	Reserve(m_nSize+1);
	m_pData[m_nSize] = dValue;
	m_nSize++;
}

// delete an element at some index
void DArray::DeleteAt(int nIndex) {
	assert(nIndex >= 0 && nIndex < m_nSize);
	// double* ptemp = new double[static_cast<size_t>(m_nSize) - 1];
	// for (int i = 0; i < nIndex; i++)
		//ptemp[i] = m_pData[i];
	// for (int i = nIndex; i < m_nSize - 1; i++)
		//ptemp[i] = m_pData[i+1];
	//delete[] m_pData;
	//m_pData = ptemp;
	for (int i = nIndex + 1; i < m_nSize; i++)
		m_pData[i - 1] = m_pData[i];
	m_nSize--;
}

// insert a new element at some index
void DArray::InsertAt(int nIndex, double dValue) {
	assert(nIndex >= 0 && nIndex <= m_nSize);
	Reserve(m_nSize+1);
	for (int i = m_nSize; i > nIndex; i--)
		m_pData[i] = m_pData[i-1];
	m_pData[nIndex] = dValue;
	m_nSize++;
}

// overload operator '='
DArray& DArray::operator = (const DArray& arr) {
	Reserve(arr.m_nSize);
	m_nSize = arr.m_nSize;
	memcpy(m_pData, arr.m_pData, arr.m_nSize * sizeof(double));
	return *this; //return reference of itself
}
