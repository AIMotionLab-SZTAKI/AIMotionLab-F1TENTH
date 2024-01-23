import logging

def get_logger():
    # Create a logger
    logger = logging.getLogger(__name__)

    # Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    logger.setLevel(logging.DEBUG)

    # Create a formatter
    formatter = logging.Formatter('[%(module)s] - %(levelname)s : %(message)s')

    # Create a console handler and set the formatter
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    # Add the console handler to the logger
    logger.addHandler(console_handler)

    return logger