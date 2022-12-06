package org.firstinspires.ftc.teamcode.utils.fileRW.main.parser;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerToken;

import java.io.IOException;
import java.util.List;

public abstract class ParserTemplate {

    public abstract ParsedFile parse(List<LexerToken> lexerTokens) throws IOException;
}