#include<time.h>
#include"config.h"

char *l_trim(char *o, const char *i)
{
  assert(i!=NULL);
  assert(o!=NULL);
  assert(i!=o);

  for(;*i!='\0' && isspace(*i);++i) ;
  return strcpy(o,i);
}

char *r_trim(char *o, const char *i)
{
  char *p=NULL;
  assert(i!=NULL);
  assert(o!=NULL);
  assert(i!=o);
  strcpy(o,i);
  for(p=o+strlen(o)-1;p>=o && isspace(*p); --p) ;
  *(++p) = '\0';
  return o;
}

char *a_trim(char *o, const char *i)
{
  char *p=NULL;
  assert(i!=NULL);
  assert(o!=NULL);
  assert(i!=o);
  l_trim(o,i);
  for(p=o+strlen(o)-1;p>=o && isspace(*p);--p) ;
  *(++p)='\0';
  return o;
}

int SetKeyInt(char *appName, char *keyName, char *profile, int Value)
{
	char ucTmp[KEYVALLEN];
	sprintf(ucTmp, "%d", Value);
	return SetKeyString(appName, keyName, profile, NULL, ucTmp);
}

//SetKeyString("server", "KEEPALIVE", "/etc/config.ini",NULL,"123");
int SetKeyString(char *appName, char *keyName, char *profile, char *keyValOld, char *keyValNew)
{
  char appname[KEYVALLEN]={0},keyname[KEYVALLEN]={0},keyvalold[KEYVALLEN]={0};
  char *buf,*c;
  char buf_i[KEYVALLEN]={0},buf_o[KEYVALLEN]={0};
  char ucFileNameBak[128]={0};
  FILE *fp,*fpnew;
  int linecount=0,tmp=0;
  int addflag=0;
  int rslt;

  time_t tp;
  struct tm *tm;

  int found=0;
  if((fp=fopen(profile,"r"))==NULL)
  {
    //printf("openfile [%s] error [%s]\n", profile, strerror(errno));
	
	fp=fopen(profile,"w");
	fclose(fp);
	
	if((fp=fopen(profile,"r"))==NULL)
	{
		return 1;
	}
	//return -1;
  }

  fseek(fp, 0, SEEK_SET);
  memset(appname, 0, sizeof(appname));
  sprintf(appname, "[%s]", appName);
  
  while(!feof(fp)&&fgets(buf_i,KEYVALLEN,fp)!=NULL)
  {
    linecount++;
    l_trim(buf_o, buf_i);
    if(strlen(buf_o)<=0)
      continue;
    buf=buf_o;
    if(found==0)
    {
      if(buf[0]!='[')
        continue;
      else if(strncmp(buf, appname, strlen(appname))==0)
      {
        found = 1;
        addflag = linecount;
        continue;
      }
	  
    }
    else if(found==1)
    {
      if(buf[0]=='#')
        continue;
      else if(buf[0]=='[')
        break;
      else
      {
        if((c=(char *)strchr(buf,'='))==NULL)
          continue;
        memset(keyname,0,sizeof(keyname));
        sscanf(buf,"%[^=|^ |^\t]",keyname);
        if(strcmp(keyname,keyName)==0)
        {
         sscanf(++c,"%[^\n]",keyvalold);
         char *keyVal_o = (char *)malloc(sizeof(keyvalold)+1);
          if(keyVal_o!=NULL)
          {
            memset(keyVal_o,0,sizeof(keyVal_o));
            a_trim(keyVal_o,keyvalold);
            if(keyVal_o&&strlen(keyVal_o)>0)
              strcpy(keyvalold,keyVal_o);
            free(keyVal_o);
            keyVal_o=NULL;
          }
          found=2;
          break;
        }
        else
          continue;
      }
    }
  }

  fclose(fp);
  time(&tp);
  tm=localtime(&tp);
  sprintf(ucFileNameBak,"%s.%04d%02d%02d%02d%02d%02d",
          profile,
          tm->tm_year+1900,
          tm->tm_mon+1,
          tm->tm_mday,
          tm->tm_hour,
          tm->tm_min,
          tm->tm_sec);
  rslt=rename(profile,ucFileNameBak);
  if(rslt<0)
  {
    printf("can not rename profile\n");
    return 2;
  }
  
  if((fp=fopen(ucFileNameBak,"r"))==NULL)
  {
    printf("can not open file:%s\n", ucFileNameBak);
    return 3;
  }

  if((fpnew=fopen(profile,"w+"))==NULL)
  {
    printf("can not open file:%s\n", profile);
    fclose(fp);
    return 4;
  }

  fseek(fp, 0, SEEK_SET);
  fseek(fpnew,0,SEEK_SET);

  switch(found)
  {
  case 0:
    while(!feof(fp)&&fgets(buf_i,KEYVALLEN,fp)!=NULL)
      fprintf(fpnew,"%s",buf_i);
    //ftime(&tp);
    //fprintf(fpnew,"#added by system %04d-%02d-%02d %02d:%02d:%02d\n",
    //        profile,
    //        tm->tm_year+1900,
    //        tm->tm_mon+1,
    //        tm->tm_mday,
    //        tm->tm_hour,
    //        tm->tm_min,
    //        tm->tm_sec);
    fprintf(fpnew,"[%s]\n%s=%s\n",appName,keyName,keyValNew);
    break;
  case 1:
    while(!feof(fp)&&fgets(buf_i,KEYVALLEN,fp)!=NULL)
    {
      fprintf(fpnew,"%s",buf_i);
      tmp++;
      if(tmp == addflag)
      {
        time (&tp);
        tm = localtime (&time);
        //fprintf(fpnew,"#add by system %04d-%02d-%02d %02d:%02d:%02d\n",
        //        tm->tm_year+1900,
        //        tm->tm_mon+1,
        //        tm->tm_mday,
        //        tm->tm_hour,
        //        tm->tm_min,
        //        tm->tm_sec);
        fprintf(fpnew,"%s=%s\n",keyName,keyValNew);
      }
    }
    break;
  case 2:
    while(!feof(fp)&&fgets(buf_i,KEYVALLEN,fp)!=NULL)
    {
      tmp++;
      if(tmp != linecount)
        fprintf(fpnew,"%s",buf_i);
      else
      {
        //fprintf(fpnew,"#%s",buf_i);
        //ftime (&tp);
        //tm = localtime (&(tp.time));
        //fprintf(fpnew,"#modify by system %04d-%02d-%02d %02d:%02d:%02d\n",
                //tm->tm_year+1900,
                //tm->tm_mon+1,
               // tm->tm_mday,
                //tm->tm_hour,
                //tm->tm_min,
                //tm->tm_sec);
        fprintf(fpnew,"%s=%s\n",keyName,keyValNew);
      }
    }
    break;
  }
  fclose(fp);
  fclose(fpnew);

  remove(ucFileNameBak);
  //char cmd[24];
  //sprintf(cmd, "rm %s", ucFileNameBak);
  //system(cmd);
  return 0;
}

void GetIniKeyString(char *appName, char *keyName, char *profile, char *keyVal)
{
  char appname[32],keyname[32];
  char *buf,*c;
  char buf_i[KEYVALLEN],buf_o[KEYVALLEN];
  FILE *fp;

  keyVal[0] = '\0';
  int found = 0;
  if((fp=fopen(profile, "r")) ==NULL)
  {
    printf("open file [%s] error %s\n", profile, strerror(errno));
    return;
  }

  fseek(fp, 0, SEEK_SET);
  memset(appname, 0, sizeof(appname));
  sprintf(appname,"[%s]", appName);
  while(!feof(fp) && fgets(buf_i,KEYVALLEN,fp)!=NULL)
  {
    l_trim(buf_o, buf_i);
    if(strlen(buf_o)<=0)
    {
      continue;
    }

    buf = NULL;
    buf = buf_o;
    if(found==0)
    {
      if(buf[0]!='[')
      {
        continue;
      }
      else if(strncmp(buf, appname, strlen(appname))==0)
      {
        found=1;
        continue;
      }
    }
    else if(found==1)
    {
      if(buf[0]=='#') continue;
      else if(buf[0]=='[') break;
      else
      {
        if((c=(char *)strchr(buf,'=')) == NULL) continue;
        memset(keyname, 0, sizeof(keyname));
        sscanf(buf, "%[^=|^ |^\t]", keyname);

        if(strcmp(keyname, keyName)==0)
        {
          sscanf(++c,"%[^\n]", keyVal);
          char *keyVal_o = (char *)malloc(strlen(keyVal)+1);
          if(keyVal_o!=NULL)
          {
            memset(keyVal_o, 0, sizeof(keyVal_o));
            a_trim(keyVal_o, keyVal);
            if(keyVal_o&& strlen(keyVal_o)>0) strcpy(keyVal, keyVal_o);
            free(keyVal_o);
            keyVal_o = NULL;
          }
          found = 2;
          break;
        }
        else continue;
      }
    }
  }
  fclose(fp);
  if(found!=2) keyVal[0]='\0';
  return;
}

int GetIniKeyInt(char *appName, char *keyName, char *profile)
{
  char ucTmp[KEYVALLEN]={0};
  GetIniKeyString(appName, keyName, profile, ucTmp);
  if(strlen(ucTmp) == 0)
  	return -1;
  return atoi(ucTmp);
}
