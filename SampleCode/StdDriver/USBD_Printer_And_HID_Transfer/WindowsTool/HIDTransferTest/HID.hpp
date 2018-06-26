#ifndef INC__HID_HPP__
#define INC__HID_HPP__

#include "stdafx.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include "dbt.h"

extern "C" {
#include "setupapi.h"
#include "hidsdi.h"
}


#define HID_MAX_PACKET_SIZE_EP 64
#define V6M_MAX_COMMAND_LENGTH (HID_MAX_PACKET_SIZE_EP - 2)

class CHidIO
{
protected:
	HANDLE m_hReadHandle;
	HANDLE m_hWriteHandle;
	HANDLE m_hReadEvent;
	HANDLE m_hWriteEvent;
	HANDLE m_hAbordEvent;
public:

	CHidIO()
		: m_hReadHandle(INVALID_HANDLE_VALUE)
		, m_hWriteHandle(INVALID_HANDLE_VALUE)
		, m_hAbordEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
		, m_hReadEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
		, m_hWriteEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
	{
	}
	virtual ~CHidIO()
	{
		CloseDevice();
		CloseHandle(m_hWriteEvent);
		CloseHandle(m_hReadEvent);
		CloseHandle(m_hAbordEvent);
	}

	void CloseDevice()
	{
		if(m_hReadHandle != INVALID_HANDLE_VALUE)
			CancelIo(m_hReadHandle);
		if(m_hWriteHandle != INVALID_HANDLE_VALUE)
			CancelIo(m_hWriteHandle);
		if(m_hReadHandle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hReadHandle);
			m_hReadHandle = INVALID_HANDLE_VALUE;
		}
		if(m_hWriteHandle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hWriteHandle);
			m_hWriteHandle = INVALID_HANDLE_VALUE;
		}
	}

	HIDP_CAPS	Capabilities;

	BOOL OpenDevice(USHORT usVID, USHORT usPID)
	{				
        DWORD                               DeviceUsage; 
		TCHAR MyDevPathName[MAX_PATH];
		//Get the Capabilities structure for the device.   	   
		PHIDP_PREPARSED_DATA    PreparsedData;   

		//�w�q�@��GUID�����c��HidGuid�ӫO�sHID�]�ƪ����f��GUID�C
		GUID HidGuid;
		//�w�q�@��DEVINFO���y�`hDevInfoSet�ӫO�s����쪺�]�ƫH�����X�y�`�C
		HDEVINFO hDevInfoSet;
		//�w�qMemberIndex�A��ܷ�e�j����ĴX�ӳ]�ơA0��ܲĤ@�ӳ]�ơC
		DWORD MemberIndex;
		//DevInterfaceData�A�ΨӫO�s�]�ƪ��X�ʱ��f�H��
		SP_DEVICE_INTERFACE_DATA DevInterfaceData;
		//�w�q�@��BOOL�ܶq�A�O�s��ƽեάO�_��^���\
		BOOL Result;
		//�w�q�@��RequiredSize���ܶq�A�Ψӱ����ݭn�O�s�ԲӫH�����w�Ī��סC
		DWORD RequiredSize;
		//�w�q�@�ӫ��V�]�ƸԲӫH�������c����w�C
		PSP_DEVICE_INTERFACE_DETAIL_DATA	pDevDetailData;
		//�w�q�@�ӥΨӫO�s���}�]�ƪ��y�`�C
		HANDLE hDevHandle;
		//�w�q�@��HIDD_ATTRIBUTES�����c���ܶq�A�O�s�]�ƪ��ݩʡC
		HIDD_ATTRIBUTES DevAttributes;
		
		//��l�Ƴ]�ƥ����
		BOOL MyDevFound=FALSE;
		
		//��l��Ū�B�g�y�`���L�ĥy�`�C
		m_hReadHandle=INVALID_HANDLE_VALUE;
		m_hWriteHandle=INVALID_HANDLE_VALUE;
		
		//��DevInterfaceData���c�骺cbSize��l�Ƭ����c��j�p
		DevInterfaceData.cbSize=sizeof(DevInterfaceData);
		//��DevAttributes���c�骺Size��l�Ƭ����c��j�p
		DevAttributes.Size=sizeof(DevAttributes);
		
		//�ե�HidD_GetHidGuid������HID�]�ƪ�GUID�A�ëO�s�bHidGuid���C
		HidD_GetHidGuid(&HidGuid);
		
		//�ھ�HidGuid������]�ƫH�����X�C�䤤Flags�ѼƳ]�m��
		//DIGCF_DEVICEINTERFACE|DIGCF_PRESENT�A�e�̪�ܨϥΪ�GUID��
		//���f��GUID�A��̪�ܥu�C�|���b�ϥΪ��]�ơA�]���ڭ̳o�̥u
		//�d��w�g�s���W���]�ơC��^���y�`�O�s�bhDevinfo���C�`�N�]��
		//�H�����X�b�ϥΧ�����A�n�ϥΨ��SetupDiDestroyDeviceInfoList
		//�P���A���M�|�y�����s���|�C
		hDevInfoSet=SetupDiGetClassDevs(&HidGuid,
			NULL,
			NULL,
			DIGCF_DEVICEINTERFACE|DIGCF_PRESENT);
		
		//AddToInfOut("�}�l�d��]��");
		//�M���]�ƶ��X���C�ӳ]�ƶi��C�|�A�ˬd�O�_�O�ڭ̭n�䪺�]��
		//����ڭ̫��w���]�ơA�Ϊ̳]�Ƥw�g�d�䧹���ɡA�N�h�X�d��C
		//�������V�Ĥ@�ӳ]�ơA�Y�NMemberIndex�m��0�C
		MemberIndex=0;
		while(1)
		{
			//�ե�SetupDiEnumDeviceInterfaces�b�]�ƫH�����X������s����
			//MemberIndex���]�ƫH���C
			Result=SetupDiEnumDeviceInterfaces(hDevInfoSet,
				NULL,
				&HidGuid,
				MemberIndex,
				&DevInterfaceData);
			
			//�p�G����H�����ѡA�h�����]�Ƥw�g�d�䧹���A�h�X�`���C
			if(Result==FALSE) break;
			
			//�NMemberIndex���V�U�@�ӳ]��
			MemberIndex++;
			
			//�p�G����H�����\�A�h�~������ӳ]�ƪ��ԲӫH���C�b����]��
			//�ԲӫH���ɡA�ݭn�����D�O�s�ԲӫH���ݭn�h�j���w�İϡA�o�q�L
			//�Ĥ@���եΨ��SetupDiGetDeviceInterfaceDetail������C�o��
			//���ѽw�İϩM���׳���NULL���ѼơA�ô��Ѥ@�ӥΨӫO�s�ݭn�h�j
			//�w�İϪ��ܶqRequiredSize�C
			Result=SetupDiGetDeviceInterfaceDetail(hDevInfoSet,
				&DevInterfaceData,
				NULL,
				NULL,
				&RequiredSize,
				NULL);
			
			//�M��A���t�@�Ӥj�p��RequiredSize�w�İϡA�ΨӫO�s�]�ƸԲӫH���C
			pDevDetailData=(PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(RequiredSize);
			if(pDevDetailData==NULL) //�p�G���s�����A�h������^�C
			{
				//MessageBox("���s����!");
				SetupDiDestroyDeviceInfoList(hDevInfoSet);
				return FALSE;
			}
			
			//�ó]�mpDevDetailData��cbSize�����c�骺�j�p�]�`�N�u�O���c��j�p�A
			//���]�A�᭱�w�İϡ^�C
			pDevDetailData->cbSize=sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
			
			//�M��A���ե�SetupDiGetDeviceInterfaceDetail��ƨ�����]�ƪ�
			//�ԲӫH���C�o���եγ]�m�ϥΪ��w�İϥH�νw�İϤj�p�C
			Result=SetupDiGetDeviceInterfaceDetail(hDevInfoSet,
				&DevInterfaceData,
				pDevDetailData,
				RequiredSize,
				NULL,
				NULL);
			
			//�N�]�Ƹ��|�ƻs�X�ӡA�M��P�����ӽЪ����s�C
			//MyDevPathName=pDevDetailData->DevicePath;
			//_tcscpy(MyDevPathName, pDevDetailData->DevicePath);
			wcscpy_s(MyDevPathName, pDevDetailData->DevicePath);
            free(pDevDetailData);
			
			//�p�G�եΥ��ѡA�h�d��U�@�ӳ]�ơC
			if(Result==FALSE) continue;
			
			//�p�G�եΦ��\�A�h�ϥΤ��aŪ�g�X�ݪ�CreateFile���
			//������]�ƪ��ݩʡA�]�AVID�BPID�B���������C
			//���@�ǿW���]�ơ]�ҦpUSB��L�^�A�ϥ�Ū�X�ݤ覡�O�L�k���}���A
			//�ӨϥΤ��aŪ�g�X�ݪ��榡�~�i�H���}�o�ǳ]�ơA�q������]�ƪ��ݩʡC
			hDevHandle=CreateFile(MyDevPathName, 
				NULL,
				FILE_SHARE_READ|FILE_SHARE_WRITE, 
				NULL,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL,
				NULL);
			
			//�p�G���}���\�A�h����]���ݩʡC
			if(hDevHandle!=INVALID_HANDLE_VALUE)
			{
				//����]�ƪ��ݩʨëO�s�bDevAttributes���c�餤
				Result=HidD_GetAttributes(hDevHandle,
					&DevAttributes);
				
				//������襴�}���]��
				//CloseHandle(hDevHandle);
				
				//������ѡA�d��U�@��
				if(Result==FALSE) continue;
				
				//�p�G������\�A�h�N�ݩʤ���VID�BPID�H�γ]�ƪ������P�ڭ̻ݭn��
				//�i�����A�p�G���@�P���ܡA�h�������N�O�ڭ̭n�䪺�]�ơC
				if(DevAttributes.VendorID == usVID
					&& DevAttributes.ProductID == usPID){
						
	                        // �Q��HID Report Descriptor�ӿ���HID Transfer�˸m 
							HidD_GetPreparsedData(hDevHandle, &PreparsedData);   
			   
							HidP_GetCaps(PreparsedData, &Capabilities);   
		
							HidD_FreePreparsedData(PreparsedData);  
							 DeviceUsage = (Capabilities.UsagePage * 256) + Capabilities.Usage;   
   
							if (DeviceUsage != 0xFF0001)   // Report Descriptor
								continue;

							MyDevFound=TRUE; //�]�m�]�Ƥw�g���
							//AddToInfOut("�]�Ƥw�g���");
							
							//����N�O�ڭ̭n�䪺�]�ơA���O�ϥ�Ū�g�覡���}���A�ëO�s��y�`
							//�åB��ܬ����B�X�ݤ覡�C
							
							//Ū�覡���}�]��
							m_hReadHandle=CreateFile(MyDevPathName, 
								GENERIC_READ,
								FILE_SHARE_READ|FILE_SHARE_WRITE, 
								NULL,
								OPEN_EXISTING,
								//FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
								FILE_ATTRIBUTE_NORMAL,
								NULL);
								//if(hReadHandle!=INVALID_HANDLE_VALUE)AddToInfOut("Ū�X�ݥ��}�]�Ʀ��\");
								//else AddToInfOut("Ū�X�ݥ��}�]�ƥ���");
							
							//�g�覡���}�]��
							m_hWriteHandle=CreateFile(MyDevPathName, 
								GENERIC_WRITE,
								FILE_SHARE_READ|FILE_SHARE_WRITE, 
								NULL,
								OPEN_EXISTING,
								//FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
								FILE_ATTRIBUTE_NORMAL,
								NULL);
								//if(hWriteHandle!=INVALID_HANDLE_VALUE)AddToInfOut("�g�X�ݥ��}�]�Ʀ��\");
								//else AddToInfOut("�g�X�ݥ��}�]�ƥ���");
							
						
							//���Ĳ�o�ƥ�A��Ū���i�u�{��_�B��C�]���b�o���e�èS���ե�
							//Ū�ƾڪ���ơA�]�N���|�ް_�ƥ󪺲��͡A�ҥH�ݭn�����Ĳ�o�@
							//���ƥ�A��Ū���i�u�{��_�B��C
							//SetEvent(ReadOverlapped.hEvent);
							
							//��ܳ]�ƪ����A�C
							//SetDlgItemText(IDC_DS,"�]�Ƥw���}");
							
							//���]�ơA�h�X�`���C���{�ǥu�˴��@�ӥؼг]�ơA�d����N�h�X
							//�d��F�C�p�G�A�ݭn�N�Ҧ����ؼг]�Ƴ��C�X�Ӫ��ܡA�i�H�]�m�@��
							//�ƲաA����N�O�s�b�Ʋդ��A����Ҧ��]�Ƴ��d�䧹���~�h�X�d��
							break;
						}
			}
			//�p�G���}���ѡA�h�d��U�@�ӳ]��
			else //continue;
			{
				CloseHandle(hDevHandle);
				continue;
			}
		}
		
		//�ե�SetupDiDestroyDeviceInfoList��ƾP���]�ƫH�����X
		SetupDiDestroyDeviceInfoList(hDevInfoSet);
	
		//�p�G�]�Ƥw�g���A�������Өϯ�U�ާ@���s�A�æP�ɸT��}�]�ƫ��s
		return MyDevFound;
	}


	BOOL ReadFile(char *pcBuffer, DWORD szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		HANDLE events[2] = {m_hAbordEvent, m_hReadEvent};

		OVERLAPPED overlapped;
		memset(&overlapped, 0, sizeof(overlapped));
		overlapped.hEvent = m_hReadEvent;

		if(pdwLength != NULL)
			*pdwLength = 0;
		
		if(!::ReadFile(m_hReadHandle, pcBuffer, szMaxLen, NULL, &overlapped))
			return FALSE;
		DWORD dwIndex = WaitForMultipleObjects(2, events, FALSE, dwMilliseconds);
		if(dwIndex == WAIT_OBJECT_0
			|| dwIndex == WAIT_OBJECT_0 + 1)
		{
			ResetEvent(events[dwIndex - WAIT_OBJECT_0]);

			if(dwIndex == WAIT_OBJECT_0)
				return FALSE;	//Abort event
			else
			{
				DWORD dwLength = 0;
				//Read OK
				GetOverlappedResult(m_hReadHandle, &overlapped, &dwLength, TRUE);
				if(pdwLength != NULL)
					*pdwLength = dwLength;
				return TRUE;
			}				
		}
		else
			return FALSE;
	}

	BOOL WriteFile(const char *pcBuffer, DWORD szLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		HANDLE events[2] = {m_hAbordEvent, m_hWriteEvent};
        
		OVERLAPPED overlapped;
		memset(&overlapped, 0, sizeof(overlapped));
		overlapped.hEvent = m_hWriteEvent;

		if(pdwLength != NULL)
			*pdwLength = 0;

		DWORD dwStart2 = GetTickCount();

		if(!::WriteFile(m_hWriteHandle, pcBuffer, szLen, NULL, &overlapped))
			return FALSE;

		DWORD dwIndex = WaitForMultipleObjects(2, events, FALSE, dwMilliseconds);
		
		if(dwIndex == WAIT_OBJECT_0
			|| dwIndex == WAIT_OBJECT_0 + 1)
		{
			ResetEvent(events[dwIndex - WAIT_OBJECT_0]);

			if(dwIndex == WAIT_OBJECT_0)
				return FALSE;	//Abort event
			else
			{
				DWORD dwLength = 0;
				//Write OK
				GetOverlappedResult(m_hWriteHandle, &overlapped, &dwLength, TRUE);
				if(pdwLength != NULL)
					*pdwLength = dwLength;
				return TRUE;
			}				
		}
		else
			return FALSE;
	}
};



class CHidCmd
{
protected:
	CHAR	m_acBuffer[HID_MAX_PACKET_SIZE_EP + 1];
	CHidIO	m_hidIO;
public:
	CHidCmd()
		: m_hidIO()
	{
	}
	virtual ~CHidCmd()
	{
	}

	void CloseDevice()
	{
		m_hidIO.CloseDevice();
	}

	BOOL OpenDevice(USHORT usVID, USHORT usPID)
	{
		return m_hidIO.OpenDevice(usVID, usPID);
	}

	BOOL ReadFile(unsigned char *pcBuffer, size_t szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
        BOOL bRet;

        bRet = m_hidIO.ReadFile(m_acBuffer, sizeof(m_acBuffer), pdwLength, dwMilliseconds);
        (*pdwLength)--;
        memcpy(pcBuffer, m_acBuffer+1, *pdwLength);

		return bRet;
	}

	BOOL WriteFile(unsigned char *pcBuffer, DWORD dwLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		/* Set new package index value */
		DWORD dwCmdLength = dwLen;
		if(dwCmdLength > sizeof(m_acBuffer) - 1)
			dwCmdLength = sizeof(m_acBuffer) - 1;
        
        memset(m_acBuffer, 0xCC, sizeof(m_acBuffer));
		m_acBuffer[0] = 0x00;	//Always 0x00
        memcpy(m_acBuffer+1  , pcBuffer, dwCmdLength);
		BOOL bRet = m_hidIO.WriteFile(m_acBuffer, 65, pdwLength, dwMilliseconds);
        if(bRet)
        {
                *pdwLength = *pdwLength - 1;
        }

        return bRet;
	}

};



#endif
