/* ===============================================================
 * Analysis of feval instructions in the code.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#ifndef ANALYSIS_FEVAL_H
#define	ANALYSIS_FEVAL_H

#include "analysismanager.h"
#include "functions.h"
#include "stmtsequence.h"
#include "typeinfer.h"

class FevalInfo : public AnalysisInfo {
    public:
        // no fields declared yet
};

AnalysisInfo* computeFevalInfo(
	const ProgFunction* pFunction,
	const StmtSequence* pFuncBody,
	const TypeSetString& inArgTypes,
	bool returnBottom
);

#endif	/* ANALYSIS_FEVAL_H */

