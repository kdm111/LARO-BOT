from .llm_ollama import _dotenv_value, _resolve_setting


def test_dotenv_value_reads_plain_export_and_quotes(tmp_path):
    path = tmp_path / '.env'
    path.write_text(
        '# comment\n'
        'PLAIN=https://plain.example\n'
        'export SINGLE=\'single-value\'\n'
        'DOUBLE="double-value"\n',
        encoding='utf-8',
    )

    assert _dotenv_value(path, 'PLAIN') == 'https://plain.example'
    assert _dotenv_value(path, 'SINGLE') == 'single-value'
    assert _dotenv_value(path, 'DOUBLE') == 'double-value'
    assert _dotenv_value(path, 'MISSING') is None


def test_workspace_dotenv_wins_over_stale_process_environment(tmp_path):
    path = tmp_path / '.env'
    path.write_text('OLLAMA_HOST=https://current.example\n', encoding='utf-8')

    value, source = _resolve_setting(
        'OLLAMA_HOST',
        'http://localhost:11434',
        env_file=path,
        environ={'OLLAMA_HOST': 'https://stale.example'},
    )

    assert value == 'https://current.example'
    assert source == path


def test_process_environment_is_fallback_when_dotenv_is_absent(tmp_path):
    missing = tmp_path / 'missing.env'

    value, source = _resolve_setting(
        'OLLAMA_HOST',
        'http://localhost:11434',
        env_file=missing,
        environ={'OLLAMA_HOST': 'https://process.example'},
    )

    assert value == 'https://process.example'
    assert source == 'process environment'
