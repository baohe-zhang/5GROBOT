from flask import current_app, Flask, redirect, url_for

def create_app(config, debug=False):
    app = Flask(__name__)
    app.config.from_object(config)
    app.debug = debug

    # Setup the data model.
    with app.app_context():
        model = get_model()
        model.init_app(app)

    # Register the status blueprint
    from .crud import crud
    app.register_blueprint(crud, url_prefix='/status')

    # Add an error handler. This is useful for debugging the live application,
    # however, you should disable the output of the exception for production
    # applications.
    @app.errorhandler(500)
    def server_error(e):
        return """
        An internal error occurred: <pre>{}</pre>
        See logs for full stacktrace.
        """.format(e), 500

    return app


def get_model():
    from . import model_cloudsql
    model = model_cloudsql

    return model

    


    

    
