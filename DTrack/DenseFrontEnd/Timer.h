#ifndef _TIMER_H_
#define _TIMER_H_

#include <iomanip>
#include <string>
#include <deque>
#include <stack>
#include <map>


class Timer {

    public:

        struct TFunction
        {
            TFunction(){
                bActive = false;
            }

            int                 nLevel;
            double              dInitTime;
            double              dChildrenTime;   // Time used by internal functions
            bool                bActive;
            std::string         sParent;         // Name of the parent function
            std::deque<double>  vProcessTime;    // Overall function time
            std::deque<double>  vAdditionalTime; // Time not accounted by dChildrenTime
        };

        Timer() 
        {
            InitReset();
        }

        ///////////////////////////////////////////////////////////////////////
        void InitReset(int nTWS = 40) 
        {
            m_mTrackedFunctions.clear();
            m_mDisplayOrder.clear();
            m_uMaxNameLength = 0;
            m_nTimerWindowSize = nTWS;

            while( !m_qFunctionStack.empty() ){
                m_qFunctionStack.pop();
            }
        }

        ///////////////////////////////////////////////////////////////////////
        void SetWindowSize( const int nTWS )
        {
            m_nTimerWindowSize = nTWS;
        }
        
        ///////////////////////////////////////////////////////////////////////
        void Tic( std::string sName="") 
        {

            // check if is main program
            if( !sName.compare("") ) {
                // reset all values
                sName = "Total";
            }

            // Look if this function is already registered
            std::map< std::string, TFunction >::iterator it;
            it = m_mTrackedFunctions.find( sName );

            if ( it==m_mTrackedFunctions.end() ) {
                TFunction f;
                f.nLevel  = (int)m_qFunctionStack.size();
                f.sParent = ( f.nLevel == 0 )?"root":m_qFunctionStack.top();
                f.bActive = false;
                m_mTrackedFunctions.insert( std::pair<std::string,TFunction>(sName,f));
                m_mDisplayOrder.insert( std::pair<int,std::string>((int)m_mDisplayOrder.size(),sName));
                if(sName.length() > m_uMaxNameLength) {
                    m_uMaxNameLength = sName.length();
                }
            }
            else if( it->second.bActive ){
                printf( "WARNING -- Timer::Tic() called with the name '%s' twice\n", sName.c_str() );
                return;
            }

            // Save current time
            m_mTrackedFunctions[sName].bActive = 1;
            m_mTrackedFunctions[sName].dInitTime = mvl::Tic();
            m_mTrackedFunctions[sName].dChildrenTime = 0.0;

            // Add function to stack
            m_qFunctionStack.push(sName);
        }

        ///////////////////////////////////////////////////////////////////////
        void Toc( std::string sName="" ) 
        {
            // check if is main program
            if( !sName.compare("") ){
                sName = "Total";
            }

            // check if there is a function in the stack
            if( (int)m_qFunctionStack.size() == 0 ) {
                std::cout << "Error [Toc]: Inbalance of Tic-Toc calls" << std::endl;
                exit(1);
            }

            // Pop last function in the stack
            m_qFunctionStack.pop();

            // Register time
            const double dProcessTime  = mvl::TocMS(m_mTrackedFunctions[sName].dInitTime);
            const double dChildrenTime = m_mTrackedFunctions[sName].dChildrenTime;
            std::string sParent              = m_mTrackedFunctions[sName].sParent;
            m_mTrackedFunctions[sName].vProcessTime.push_back(dProcessTime);
            m_mTrackedFunctions[sName].vAdditionalTime.push_back(dProcessTime-dChildrenTime);
            m_mTrackedFunctions[sName].bActive = 0;
            if( sParent.compare("root") ){
                m_mTrackedFunctions[sParent].dChildrenTime += dProcessTime;
            }

            // Delete oldest time from the vector if we have more than needed
            if((int)m_mTrackedFunctions[sName].vProcessTime.size() > m_nTimerWindowSize) {
                m_mTrackedFunctions[sName].vProcessTime.pop_front();
                m_mTrackedFunctions[sName].vAdditionalTime.pop_front();
            }
        }

        ///////////////////////////////////////////////////////////////////////
        // return number of registered functions with level <= nLevel
        int GetNumFunctions(int nLevel)
        {
            int nNumFunc = 0;
            std::map< std::string, TFunction >::iterator it;

            for( it=m_mTrackedFunctions.begin(); it!=m_mTrackedFunctions.end(); ++it )
                if( (*it).second.nLevel <= nLevel )
                    nNumFunc++;

            return nNumFunc;
        }

        ///////////////////////////////////////////////////////////////////////
        int GetWindowSize() 
        {
            return m_nTimerWindowSize;
        }

        ///////////////////////////////////////////////////////////////////////
        void PrintToTerminal( int nLevels=0 ) 
        {

            int nLevel;
            double dProcessTime;
            std::map< int, std::string >::iterator it;
            std::string sFuncName;

            std::cout << "-----------------------------------------------" << std::endl;
            for( it = m_mDisplayOrder.begin(); it != m_mDisplayOrder.end(); ++it) {
                sFuncName = (*it).second;
                nLevel    = m_mTrackedFunctions[sFuncName].nLevel;
                if( nLevel <= nLevels ) {
                    dProcessTime = m_mTrackedFunctions[sFuncName].vProcessTime.back();
                    if(nLevel > 1) {
                        std::cout << std::setw( 5 * (m_mTrackedFunctions[sFuncName].nLevel-1) ) << "";
                    }
                    std::cout << std::setw( m_uMaxNameLength + 5 ) << std::left << (*it).second;
                    std::cout << std::setw(10) << std::setprecision(3) << std::fixed << std::right;
                    std::cout << dProcessTime << std::endl;
                }
            }
        }

        ///////////////////////////////////////////////////////////////////////
        std::vector<std::string> GetNames( const int nLevels ) 
        {
            std::vector<std::string> vNames;
            vNames.resize( GetNumFunctions( nLevels ) );
            
            int nLevel;
            std::string  sFuncName;
            std::string  sLevelMark;

            std::map< int, std::string>::iterator it;
            
            int kk = 0;
            for(it=m_mDisplayOrder.begin(); it!=m_mDisplayOrder.end(); ++it ) {
            
                sFuncName  = (*it).second;
                nLevel     = m_mTrackedFunctions[sFuncName].nLevel;
                sLevelMark = "";

                for( int ii=1; ii < nLevel; ++ii)  sLevelMark += "    ";

                if( nLevel > 0 ) sLevelMark += "|-- ";

                if( nLevel <= nLevels ) {
                    if( (nLevel < nLevels) && (m_mTrackedFunctions[sFuncName].dChildrenTime > 0.0) ) {
                        // This is a parent function with children in the last level
                        vNames[kk] = sLevelMark + sFuncName + " [T]";
                    }else{
                        // This is either a function in the last requested level or a parent function
                        // with no children
                        vNames[kk] = sLevelMark + sFuncName;
                    }
                    kk++;
                }
            }
            return vNames;
        }


        ///////////////////////////////////////////////////////////////////////
        std::vector< std::pair<double,double> > GetTimes( const int nLevels ) 
        {
            std::vector< std::pair<double,double> > vTimes;
            vTimes.resize( GetNumFunctions( nLevels ) );
            
            int nLevel;
            double  dProcessTime;
            double  dAdditionalTime;
            std::string  sFuncName;
            std::map< int, std::string>::iterator it;
           
            int kk=0;
            for( it=m_mDisplayOrder.begin(); it!=m_mDisplayOrder.end(); ++it ) {
                
                sFuncName = (*it).second;
                nLevel    = m_mTrackedFunctions[sFuncName].nLevel;

                if( nLevel <= nLevels ) {
                    dProcessTime    = m_mTrackedFunctions[sFuncName].vProcessTime.back();
                    dAdditionalTime = m_mTrackedFunctions[sFuncName].vAdditionalTime.back();

                    if( (nLevel < nLevels) && (m_mTrackedFunctions[sFuncName].dChildrenTime > 0.0) ) {
                        // This is a parent function with children in the last level
                        vTimes[kk] = std::pair<double,double>(dProcessTime,dAdditionalTime);
                    } 
                    else {
                        // This is either a function in the last requested level or a parent function
                        // with no children
                        vTimes[kk] = std::pair<double,double>(dProcessTime,dProcessTime);
                    }
                    kk++;
                }
            }
            return vTimes;
        }

    private:

        unsigned int                               m_uMaxNameLength;
        int                                        m_nTimerWindowSize;
        std::stack< std::string >                  m_qFunctionStack;
        std::map< std::string,TFunction >          m_mTrackedFunctions;
        std::map< int, std::string >               m_mDisplayOrder;
};

#endif // _TIMER_H_
